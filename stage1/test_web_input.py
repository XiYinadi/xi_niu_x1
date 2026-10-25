"""Actual local UDP ingress handler with fake PWM output."""
import ast
import math
from pathlib import Path
import threading
import time
import unittest


class InputTests(unittest.TestCase):
    def test_owner_age_finite_and_disarm_gates(self):
        p=Path(__file__).resolve().parent.parent/'slam_viz.py'
        if not p.exists():p=Path.home()/'slam_viz.py'
        tree=ast.parse(p.read_text())
        nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('drive_packet_fresh','_stage1_drive_packet')]
        out=[]
        scope=dict(_drive_io_lock=threading.RLock(),_motor_armed=True,_drive_owner='owner',time=time,math=math,
                   _send_drive=lambda *x:out.append(x))
        def stop():scope['_motor_armed']=False;scope['_drive_owner']=None;out.append('disarmed')
        scope['_navigation_disarm']=stop
        exec(compile(ast.Module(body=nodes,type_ignores=[]),str(p),'exec'),scope)
        handle=scope['_stage1_drive_packet']
        handle(dict(sid='other',l=.1,a=0,sent_monotonic=time.monotonic()))
        self.assertEqual(out,[])
        handle(dict(sid='owner',l=.1,a=0,sent_monotonic=time.monotonic()))
        self.assertEqual(out[-1],(.1,0.,True))
        handle(dict(sid='owner',l=.1,a=0,sent_monotonic=time.monotonic()-1))
        self.assertEqual(out[-1],'disarmed')
        count=len(out);handle(dict(sid='owner',l=.1,a=0,sent_monotonic=time.monotonic()))
        self.assertEqual(len(out),count)
        for value in (float('nan'),2,None):
            scope['_motor_armed']=True;scope['_drive_owner']='owner'
            handle(dict(sid='owner',l=value,a=0,sent_monotonic=time.monotonic()))
            self.assertFalse(scope['_motor_armed'])


if __name__=='__main__':unittest.main()
