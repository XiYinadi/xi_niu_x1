"""Single mounting transform used by odometry, TF and diagnostic clearance."""
import argparse
import json
import math
from pathlib import Path


def load_mount(path=None):
    path=Path(path) if path else Path(__file__).resolve().with_name('laser_mounting.json')
    data=json.loads(path.read_text())
    keys=('x_m','y_m','z_m','yaw_rad')
    if not all(type(data.get(k)) in (int,float) and math.isfinite(data[k]) for k in keys):
        raise ValueError('Invalid laser mounting transform')
    return {k:float(data[k]) for k in keys}


def base_point(x,y,mount):
    c,s=math.cos(mount['yaw_rad']),math.sin(mount['yaw_rad'])
    return c*x-s*y+mount['x_m'],s*x+c*y+mount['y_m']


def rebase_pose(pose, old_mount, new_mount):
    """Change the body reference while preserving the physical laser map pose."""
    values=[pose[k] for k in ('x','y','yaw')]
    values += [m[k] for m in (old_mount,new_mount) for k in ('x_m','y_m','yaw_rad')]
    if not all(type(v) in (int,float) and math.isfinite(v) for v in values):
        raise ValueError('Invalid pose or mounting transform')
    old_yaw=pose['yaw']
    new_yaw=old_yaw+old_mount['yaw_rad']-new_mount['yaw_rad']
    c,s=math.cos(old_yaw),math.sin(old_yaw)
    laser_x=pose['x']+c*old_mount['x_m']-s*old_mount['y_m']
    laser_y=pose['y']+s*old_mount['x_m']+c*old_mount['y_m']
    c,s=math.cos(new_yaw),math.sin(new_yaw)
    return dict(x=laser_x-c*new_mount['x_m']+s*new_mount['y_m'],
                y=laser_y-s*new_mount['x_m']-c*new_mount['y_m'],
                yaw=math.atan2(math.sin(new_yaw),math.cos(new_yaw)))


def tf_arguments(mount):
    return ['--x',str(mount['x_m']),'--y',str(mount['y_m']),'--z',str(mount['z_m']),
            '--yaw',str(mount['yaw_rad']),'--pitch','0','--roll','0',
            '--frame-id','base_link','--child-frame-id','laser']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--tf-args',action='store_true');args=p.parse_args()
    mount=load_mount()
    print(' '.join(tf_arguments(mount)) if args.tf_args else json.dumps(mount))
