"""Exercise actual PWM tick with fake I/O: diagnostic steering + safety gates."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
import time
import unittest
import sys
ROOT=Path(__file__).resolve().parent.parent
SOURCE=ROOT/'pwm_proc.py' if (ROOT/'pwm_proc.py').exists() else Path.home()/'pwm_proc.py'
sys.path.insert(0,str(SOURCE.resolve().parent))
from motor_control import Deadman, IdlePark, joy_to_channels


class PwmModeTests(unittest.TestCase):
    def test_center_hold_and_shared_disarm(self):
        source=SOURCE
        tree=ast.parse(source.read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PwmProc')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_step')
        scope=dict(time=time,joy_to_channels=joy_to_channels)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        outputs=[];servo=[];now=time.monotonic()
        gate=Deadman(.3);gate.update(0,now-1);gate.update(0,now)
        obj=S(_last_pkt=now,_deadman_gate=gate,_stale=False,_deadman=.3,
              _idle_park=IdlePark(1.),_l=0.,_a=.3,_armed=True,_a_to_wheels=True,
              _a_to_servo=True,_hold_center=True,_max_lin=.3,_max_ang=1.,_wheel_base=.2,
              _swap_lr=True,_max_duty=.1,_deadband=0.,_flip_l=False,_flip_r=False,
              _dir_l=True,_dir_r=False,_pwm_freq=1000.,
              hw=S(set_duty=lambda *v:outputs.append(v),set_direction=lambda *v:None),
              servo=S(park=lambda:servo.append('park'),set_command=servo.append),_log=lambda *v:None)
        scope['_step'](obj)
        self.assertEqual(servo[-1],0.)
        self.assertAlmostEqual(outputs[-1][0],.1);self.assertAlmostEqual(outputs[-1][1],.1)
        self.assertNotEqual(obj._dir_l,obj._dir_r)
        obj._armed=False;scope['_step'](obj)
        self.assertEqual(outputs[-1],(0.,0.));self.assertEqual(servo[-1],'park')
        obj._armed=True;obj._last_pkt=now-1;scope['_step'](obj)
        self.assertEqual(outputs[-1],(0.,0.));self.assertEqual(servo[-1],'park')


if __name__=='__main__':unittest.main()
