"""Timestamped gyro integration and gyro-constrained 2-D laser registration."""
import math
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def calibrate(samples):
    """Require a stationary, gravity-bearing sensor; return bias and up axis."""
    a = np.asarray(samples, dtype=float)
    if a.ndim != 2 or a.shape[0] < 100 or a.shape[1] != 6 or not np.isfinite(a).all():
        raise ValueError('Need at least 100 finite accel/gyro samples')
    accel, gyro = a[:, :3], a[:, 3:]
    gravity = accel.mean(axis=0)
    if not 8.0 < np.linalg.norm(gravity) < 11.5:
        raise ValueError('Gravity out of range; check sensor installation')
    if np.max(accel.std(axis=0)) > .25 or np.max(gyro.std(axis=0)) > .012 or np.linalg.norm(gyro.mean(axis=0)) > .10:
        raise ValueError('Sensor moving during calibration; keep chassis stationary')
    return gyro.mean(axis=0), gravity / np.linalg.norm(gravity)


class GyroHistory:
    def __init__(self, max_gap=.15):
        self.samples = deque(maxlen=2000)
        self.max_gap = max_gap

    def add(self, stamp, rate):
        if not math.isfinite(stamp) or not math.isfinite(rate):
            return
        if self.samples and stamp <= self.samples[-1][0]:
            self.samples.clear()  # Clock reversal must not bridge old data.
        self.samples.append((stamp, rate))

    def delta(self, start, end):
        """Integrate exact scan interval; missing coverage means laser fallback."""
        if end <= start or not self.samples:
            return None
        rows = list(self.samples)
        if rows[0][0] > start or rows[-1][0] < end:
            return None
        total = 0.
        for (ta, wa), (tb, wb) in zip(rows, rows[1:]):
            left, right = max(start, ta), min(end, tb)
            if right <= left:
                continue
            if tb - ta > self.max_gap:
                return None
            wl = wa + (wb-wa)*(left-ta)/(tb-ta)
            wr = wa + (wb-wa)*(right-ta)/(tb-ta)
            total += (wl+wr)*.5*(right-left)
        return total


class MotionDiscontinuity(ValueError):
    """An untrusted scan step requires a stationary restart, not interpolation."""


class MotionRejected(ValueError):
    """No output for this scan; recheck against the unchanged trusted reference."""


class KeyframeMotion:
    """Pose relative to a retained scan, not a sum of every noisy scan pair.

    A new keyframe is chosen after 8cm, 0.05rad or 2s. Every accepted scan still
    produces its measured pose, including small motion; no zero-motion deadband.
    Velocity uses the previous publication interval, not the keyframe interval.
    """
    def __init__(self, stamp, pose=(0.,0.,0.), max_speed_mps=.5, noise_allowance_m=.01):
        # This is a supported estimation envelope, not a measured motor speed.
        # It exceeds the supervised 0.08m/s trials and the 0.10m/s Nav2 cap.
        if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in (max_speed_mps,noise_allowance_m)):
            raise ValueError('Invalid laser motion envelope')
        self.max_speed_mps=max_speed_mps
        self.noise_allowance_m=noise_allowance_m
        self.fault=None
        self.recheck=None
        self.pose=tuple(pose)
        self.reference_pose=tuple(pose)
        self.last_stamp=self.reference_stamp=float(stamp)

    def update(self, dx, dy, theta, stamp):
        if self.fault is not None:
            raise MotionDiscontinuity(self.fault)
        if not all(math.isfinite(v) for v in (dx,dy,theta,stamp)) or stamp<=self.last_stamp:
            raise ValueError('Invalid registration result or non-increasing stamp')
        x,y,yaw=self.reference_pose
        c,s=math.cos(yaw),math.sin(yaw)
        new=(x+c*dx-s*dy,y+s*dx+c*dy,math.atan2(math.sin(yaw+theta),math.cos(yaw+theta)))
        dt=stamp-self.last_stamp
        px,py,pa=self.pose
        vx,vy=new[0]-px,new[1]-py
        distance=math.hypot(vx,vy)
        limit=self.max_speed_mps*dt+self.noise_allowance_m
        if self.recheck is not None:
            # Never make an originally impossible jump plausible by waiting.
            # Two subsequent measurements must fit the ORIGINAL envelope and
            # each other, within 0.4s of the last trusted publication.
            if stamp<=self.recheck['stamp']:
                raise ValueError('Non-increasing recheck stamp')
            self.recheck['stamp']=stamp
            if dt>.4:
                self.fault='Laser recheck expired; odom suspended, stop and restart localization at rest'
                raise MotionDiscontinuity(self.fault)
            limit=self.recheck['limit']
        if distance>limit:
            self.recheck=dict(limit=limit,stamp=stamp,candidate=None)
            raise MotionRejected(f'Untrusted laser translation {distance:.3f}m in {dt:.3f}s '
                                 f'exceeds supported step {limit:.3f}m; no output, rechecking trusted reference')
        if self.recheck is not None:
            candidate=self.recheck['candidate']
            self.recheck['candidate']=(new,stamp)
            if candidate is None:
                raise MotionRejected('Laser recheck: waiting for second consistent scan; no output')
            prior,prior_stamp=candidate
            step=math.hypot(new[0]-prior[0],new[1]-prior[1])
            if step>self.max_speed_mps*(stamp-prior_stamp)+self.noise_allowance_m:
                raise MotionRejected('Laser recheck: candidates disagree; no output')
            self.recheck=None
        c,s=math.cos(new[2]),math.sin(new[2])
        velocity=((c*vx+s*vy)/dt,(-s*vx+c*vy)/dt,
                  math.atan2(math.sin(new[2]-pa),math.cos(new[2]-pa))/dt)
        advance=math.hypot(dx,dy)>=.08 or abs(theta)>=.05 or stamp-self.reference_stamp>=2.
        self.pose,self.last_stamp=new,stamp
        if advance:self.reference_pose,self.reference_stamp=new,stamp
        return new,velocity,advance


def align(source, target, normals, gyro_delta=None, iterations=15, gate=.6):
    """Point-to-plane ICP, with a soft yaw constraint in radians when available.

    Align current scan to previous scan: positive gyro yaw yields positive R.
    A missing IMU drops only the prior. No accelerometer double integration.
    """
    theta = 0. if gyro_delta is None else gyro_delta
    trans = np.zeros(2)
    tree = cKDTree(target)
    error = float('inf')
    for _ in range(iterations):
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        rotated = source @ rotation.T
        current = rotated + trans
        distance, index = tree.query(current)
        mask = (distance < gate) & np.isfinite(normals[index]).all(axis=1)
        if mask.sum() < 20:
            return None
        p, q, n = rotated[mask], target[index[mask]], normals[index[mask]]
        residual = (n*(q-current[mask])).sum(axis=1)
        # Robust residual weighting rejects isolated range outliers.
        weights = np.sqrt(np.minimum(1., .05/np.maximum(np.abs(residual), 1e-9)))
        A = np.column_stack((n[:, 0], n[:, 1], n[:, 1]*p[:, 0]-n[:, 0]*p[:, 1]))
        A, b = A*weights[:, None], residual*weights
        if gyro_delta is not None:
            strength = math.sqrt(mask.sum()) * 1.0  # One-metre yaw lever arm.
            A = np.vstack((A, [0., 0., strength]))
            b = np.append(b, strength*(gyro_delta-theta))
        step, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
        if rank < 3 or not np.isfinite(step).all():
            return None  # Do not claim motion in unobservable geometry.
        trans += step[:2]
        theta += float(step[2])
        error = float(np.mean(np.abs(residual)))
        if np.linalg.norm(step) < 1e-5:
            break
    if error > .12 or np.linalg.norm(trans) > .4 or abs(theta) > .6:
        return None
    return float(trans[0]), float(trans[1]), theta, error
