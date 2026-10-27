"""Bounded ROS velocity feedback and acceptance geometry. No ROS/GPIO imports."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return dict(forward_m=c*dx+s*dy, lateral_m=-s*dx+c*dy,
                displacement_m=math.hypot(dx,dy), yaw_deg=math.degrees(wrap(end[2]-start[2])))


class Feedback:
    """No assumed full-throttle speed. Feedback stays within a hard duty cap.

    Gains are diagnostic controller settings, not physical calibration values.
    A zero target bypasses PI immediately. Missing feedback is handled by bridge.
    """
    def __init__(self, k, duty_cap=.10):
        if not math.isfinite(k) or k<=0 or not 0<duty_cap<=.55:
            raise ValueError('invalid conversion or diagnostic duty cap')
        self.k, self.cap = k, duty_cap
        self.reset()

    def reset(self):
        self.iv=self.iw=self.l=self.a=0.

    def step(self, v, w, measured_v, measured_w, dt, diff=False):
        if not all(math.isfinite(x) for x in (v,w,measured_v,measured_w,dt)):
            raise ValueError('non-finite command/feedback')
        # 2026-10-27: 巡航速度已提到 0.28 m/s，这里的上限必须跟着放（原来 0.25），
        # 否则任何快于 0.25 的指令都会抛错、巡检直接停。报错文案也同步成真实值。
        if not 0<=v<=.35 or abs(w)>.60:
            raise ValueError('diagnostic speed limits: 0..0.35m/s, +/-0.60rad/s')
        if not diff and abs(w)>1e-6:
            raise ValueError('angular test requires explicit temporary differential mode')
        dt=max(0.,min(dt,.04))
        if v==0 and w==0:
            self.reset();return 0.,0.
        if v==0:self.iv=0.;l=0.
        else:
            self.iv=max(0.,min(self.cap,self.iv+.6*(v-measured_v)*dt))
            l=max(0.,min(self.cap,1.2*(v-measured_v)+self.iv))
        if w==0:self.iw=0.;a=0.
        else:
            bound=self.cap/self.k
            self.iw=max(-bound,min(bound,self.iw+.45*(w-measured_w)*dt))
            a=max(-bound,min(bound,.6*(w-measured_w)+self.iw))
        # Shared cap applies to BOTH l +/- k*a, not independently to two axes.
        scale=max(1.,(abs(l)+abs(self.k*a))/self.cap)
        l,a=l/scale,a/scale
        self.l+=max(-.08*dt,min(.08*dt,l-self.l))
        self.a+=max(-.20*dt/self.k,min(.20*dt/self.k,a-self.a))
        scale=max(1.,(abs(self.l)+abs(self.k*self.a))/self.cap)
        self.l,self.a=self.l/scale,self.a/scale
        return self.l,self.a
