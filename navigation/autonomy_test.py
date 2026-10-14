"""Mission sequencing uses in-memory navigation spies: never touches hardware."""
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'inspection'))
sys.path.extend(['/usr/local/lib/python3.10/dist-packages','/usr/lib/python3/dist-packages'])
from autonomy_runtime import AutonomyRuntime
from navigation_test import config
from inspection_service import InspectionService,statistics
from inspection_test import sample


class Missions(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();root=Path(self.temp.name)
        self.task=dict(name='test',map='room',cycles=1,waypoints=[dict(x=1.,y=1.,yaw=0.,dwell_s=0)])
        self.inspection=InspectionService(root/'data'/'inspections',task_get=lambda n:self.task,sample=sample,
            analyze=lambda s,d:dict(summary='仅整理实际检测记录',recommendations=[]))
        self.nav=nav=S(root=root,lock=threading.RLock(),config=config(),mode='localization',
            armed=lambda:True,owner=lambda:'driver',localized=lambda t:True,busy=False,pending=False,navigating=False,
            nav=Mock(),lifecycle={n:3 for n in ('planner_server','controller_server','bt_navigator','velocity_smoother')},
            active_map='room',last_pose=dict(x=1.,y=1.,yaw=0),
            node=S(_lock=threading.RLock(),_last_scan_t=time.monotonic()),
            patrols=S(get=lambda name:self.task),check_goal=Mock(),pose_message=lambda p:p,
            send_goal=Mock(),send_drive=Mock(),disarm=Mock(),cancel=Mock(),socketio=Mock())
        self.r=AutonomyRuntime(nav,self.inspection)
        self.r.camera=sample(time.time()-.1);self.r.camera['capture_ok']=True
        self.r.camera_received=time.monotonic();self.r.camera_error=None

    def tearDown(self):
        if self.r.inspection_id:
            end=time.monotonic()+3
            while time.monotonic()<end:
                with self.inspection.lock:
                    if self.inspection.get(self.r.inspection_id)['status'] not in ('sampling','reporting'):break
                time.sleep(.01)
        self.temp.cleanup()

    def start(self):self.r.control(dict(action='patrol_start',name='test'),'driver')

    def test_start_requires_calibration(self):
        self.nav.config['calibrated']=False
        with self.assertRaises(ValueError):self.start()
        self.nav.send_goal.assert_not_called();self.nav.send_drive.assert_not_called()

    def test_nonowner_cannot_start(self):
        with self.assertRaises(ValueError):self.r.control(dict(action='patrol_start',name='test'),'other')
        self.assertFalse(self.r.active)

    def test_stale_camera_blocks_start(self):
        self.r.camera['captured_at']-=30
        with self.assertRaises(ValueError):self.start()

    def test_map_mismatch_blocks(self):
        self.task['map']='other'
        with self.assertRaises(ValueError):self.start()

    def test_heartbeat_loss_stops_during_dwell(self):
        self.start();self.r.state['status']='dwell';self.r.last_client-=1;self.r.tick()
        self.assertFalse(self.r.active);self.nav.send_drive.assert_called_with(0.,0.,False)
        self.nav.disarm.assert_called();self.assertTrue(self.r.report_pending)

    def test_alarm_stops_before_persisting_evidence(self):
        self.start();self.r.raise_alert(dict(self.r.camera,fire=True,reason='发现明火'))
        self.assertFalse(self.r.active);self.assertEqual(self.r.state['status'],'alert')
        alert=self.r.alert;self.assertTrue((self.r.root/alert['image']).is_file())
        session=self.inspection.get(self.r.inspection_id)
        self.assertEqual(statistics(session)['suspected'],1)
        self.assertTrue((self.inspection.directory(session['id'])/alert['image']).is_file())
        self.nav.send_drive.assert_called_with(0.,0.,False)

    def test_acknowledge_does_not_resume(self):
        self.start();self.r.raise_alert(dict(self.r.camera,fire=True,reason='发现明火'))
        self.r.control(dict(action='alert_ack'),'driver')
        self.assertFalse(self.r.active);self.assertTrue(self.r.alert['acknowledged_at'])

    def test_late_arrival_after_stop_cannot_capture(self):
        self.start();self.r.tick();done=self.nav.send_goal.call_args.kwargs['on_result']
        self.r.halt('stop');done([],None)
        self.assertEqual(self.inspection.get(self.r.inspection_id)['records'],[])

    def test_arrival_must_match_actual_pose(self):
        self.start();self.r.tick();done=self.nav.send_goal.call_args.kwargs['on_result']
        self.nav.last_pose=dict(x=9,y=9,yaw=0);done([],None)
        self.assertEqual(self.r.state['status'],'blocked');self.assertFalse(self.r.active)

    def test_full_patrol_stops_then_captures_then_reports(self):
        self.start();self.r.tick();self.nav.send_goal.call_args.kwargs['on_result']([],None)
        self.assertEqual(self.r.state['status'],'dwell');self.r.wait_until=0;self.r.tick()
        end=time.monotonic()+3
        while self.inspection.get(self.r.inspection_id)['status'] in ('sampling','reporting') and time.monotonic()<end:time.sleep(.01)
        self.r.tick();self.assertFalse(self.r.active)
        session=self.inspection.get(self.r.inspection_id)
        self.assertEqual(session['next_step'],1);self.assertEqual(session['mode'],'automatic')
        self.assertTrue(session['records'][0]['navigation_confirmed_arrival'])
        self.assertFalse(session['records'][0]['operator_confirmed_arrival'])

    def test_restart_does_not_resume(self):
        self.start();restored=AutonomyRuntime(self.nav,self.inspection)
        self.assertFalse(restored.active);self.assertEqual(restored.state['status'],'interrupted')

    def test_camera_loss_stops_between_goals(self):
        self.start();self.r.camera_received-=5;self.r.tick()
        self.assertFalse(self.r.active);self.nav.send_goal.assert_not_called()

    def test_scan_loss_stops_between_goals(self):
        self.start();self.nav.node._last_scan_t-=1;self.r.tick()
        self.assertFalse(self.r.active);self.nav.send_goal.assert_not_called()

    def test_final_uncertain_detection_is_not_completed_clean(self):
        self.start();sid=self.r.inspection_id;s=self.inspection.get(sid)
        s['status']='completed';s['records']=[dict(id='a'*32,status='ok',step=0,verdict='uncertain',fire=False,
            confidence=.6,reason='待复核',captured_at=time.time(),image='a.jpg')]
        self.inspection.directory(sid).joinpath('a.jpg').write_bytes(b'\xff\xd8test\xff\xd9')
        self.r.state['status']='detecting';self.r.tick()
        self.assertEqual(self.r.state['status'],'alert')
        self.assertEqual(statistics(s)['uncertain'],1)

    def test_timeout_cancels_stuck_goal(self):
        self.start();self.r.goal_started=time.monotonic()-121;self.nav.navigating=True
        self.r.tick();self.assertFalse(self.r.active);self.assertEqual(self.r.state['status'],'blocked')

    def test_restart_finishes_stopped_but_unarchived_inspection(self):
        self.start();sid=self.r.inspection_id;self.r.halt('stop')
        restored=AutonomyRuntime(self.nav,self.inspection)
        self.assertFalse(restored.active);self.assertTrue(restored.report_pending)
        restored.tick()
        end=time.monotonic()+3
        while self.inspection.get(sid)['status']=='reporting' and time.monotonic()<end:time.sleep(.01)
        self.assertEqual(self.inspection.get(sid)['status'],'completed')

    def test_unarchived_report_blocks_new_mission(self):
        self.start();self.r.halt('stop')
        with self.assertRaises(ValueError):self.start()


if __name__=='__main__':unittest.main()
