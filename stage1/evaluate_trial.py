"""Attach independent physical measurements to a trial. Never use odom as truth."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial');p.add_argument('--distance-m',type=float,required=True)
    p.add_argument('--yaw-deg',type=float,required=True)
    p.add_argument('--lateral-m',type=float,required=True)
    p.add_argument('--notes',required=True)
    args=p.parse_args();path=Path(args.trial);data=json.loads(path.read_text())
    import math
    if not all(math.isfinite(v) for v in (args.distance_m,args.yaw_deg,args.lateral_m)):
        raise ValueError('Measurements must be finite')
    odom=data['odometry']
    if odom is None:raise ValueError('Trial has no odometry')
    data['ground_truth']=dict(forward_m=args.distance_m,yaw_deg=args.yaw_deg,lateral_m=args.lateral_m,notes=args.notes)
    data['errors']=dict(forward_m=odom['forward_m']-args.distance_m,yaw_deg=odom['yaw_deg']-args.yaw_deg,
                        lateral_m=odom['lateral_m']-args.lateral_m)
    data['criteria']=dict(distance_tolerance_m=.05,yaw_tolerance_deg=5.,lateral_tolerance_m=.05)
    ok=abs(data['errors']['forward_m'])<=.05 and abs(data['errors']['yaw_deg'])<=5
    if data['kind']=='one_meter':
        ok=ok and abs(args.distance_m-1.)<=.05 and abs(args.yaw_deg)<=5 and abs(args.lateral_m)<=.05
    elif data['kind']=='ninety_deg':
        ok=ok and abs(args.yaw_deg-90.)<=5 and math.hypot(args.distance_m,args.lateral_m)<=.05
    else:ok=False  # Probes never count as full acceptance.
    data['accepted']=ok and data['reason']=='odometry target reached'
    out=path.with_name(path.stem+'-measured.json');out.write_text(json.dumps(data,indent=2))
    print(json.dumps({k:v for k,v in data.items() if k!='samples'},ensure_ascii=False))


if __name__=='__main__':main()
