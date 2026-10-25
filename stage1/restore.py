#!/usr/bin/env python3
"""Explicit cleanup: stop diagnostic bridge and restore original motor parameters.

Run on the board as aidlux. sudo is invoked normally; no stored password here.
Does not delete maps, logs or trial results. Leaves mapping running for inspection.
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
for p in ('/usr/local/lib/python3.10/dist-packages','/usr/lib/python3/dist-packages'):
    if p not in sys.path:sys.path.append(p)
import socketio
import yaml


def main():
    here=Path(__file__).resolve().parent
    backup=Path((here/'latest_backup.txt').read_text().strip())
    meta=json.loads((backup/'metadata.json').read_text())
    target=Path(meta['paths']['motor_params.yaml'])
    if not target.resolve().is_relative_to(Path('/home/aidlux/robot')):
        raise ValueError('Unexpected configuration path')
    original=yaml.safe_load((backup/'motor_params.yaml').read_text())
    current=yaml.safe_load(target.read_text())
    expected=json.loads(json.dumps(original))
    expected['motor_driver']['ros__parameters']['max_duty']=.1
    diff=json.loads(json.dumps(expected))
    diff['motor_driver']['ros__parameters'].update(steer_mode='diff',servo_hold_center=True)
    if current not in (original,expected,diff):
        raise ValueError('Motor parameters changed outside the trial; review before restoring')
    sio=socketio.Client(reconnection=False)
    sio.connect('http://127.0.0.1:5800?view=control',transports=['polling'])
    sio.call('motor_control',dict(action='estop'),timeout=3)
    sio.disconnect()
    pids=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:args=(p/'cmdline').read_bytes().split(b'\0')
        except (OSError,ProcessLookupError):continue
        if len(args)>1:
            script=Path(os.fsdecode(args[1]))
            try:script=(p/'cwd').resolve()/script if not script.is_absolute() else script
            except OSError:continue
            if script.resolve()==here/'ros_bridge.py':
                pids.append(int(p.name));os.kill(int(p.name),signal.SIGTERM)
    deadline=time.monotonic()+4
    while time.monotonic()<deadline and any(Path('/proc',str(pid)).exists() for pid in pids):time.sleep(.05)
    for pid in pids:
        stat=Path('/proc',str(pid),'stat')
        if stat.exists() and stat.read_text().split()[2]!='Z':raise RuntimeError('Bridge did not exit')
    tmp=target.with_suffix('.stage1-restore')
    tmp.write_bytes((backup/'motor_params.yaml').read_bytes());tmp.replace(target)
    subprocess.run(['sudo','systemctl','restart','rhino-pwm'],check=True)
    if meta['gamepad_active']:subprocess.run(['sudo','systemctl','start','rhino-gamepad'],check=True)
    print('Restored original motor parameters; PWM starts disarmed. Trial records preserved.')


if __name__=='__main__':main()
