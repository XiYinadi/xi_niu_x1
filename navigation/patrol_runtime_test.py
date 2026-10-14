"""Exercise actual navigation action callbacks without a ROS executor or outputs."""
import threading
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
from geometry_msgs.msg import PoseStamped
from navigation_runtime import NavigationRuntime
from patrol_preview import PatrolPreview
from patrol_test import task


class Future:
    def add_done_callback(self, callback): self.callback=callback
    def result(self):return self.value
    def resolve(self, value):self.value=value;self.callback(self)


class Runtime(unittest.TestCase):
    def setUp(self):
        r=self.r=NavigationRuntime.__new__(NavigationRuntime)
        r.lock=threading.RLock();r.pending=False;r.navigating=False;r.epoch=0
        r.route_preview=PatrolPreview();r.goal_handle=None;r.active_map='room'
        r.last_pose=dict(x=0.,y=0.,yaw=0.);r.localized=lambda now:True
        r.running=lambda:True;r.check_goal=Mock();r.send_drive=Mock();r.disarm=Mock()
        r.armed=lambda:False;r.owner=lambda:None
        r.lifecycle={'planner_server':3};r.planner=Mock();r.nav=Mock()
        self.future=Future();r.planner.send_goal_async.return_value=self.future
        def pose(data):
            p=PoseStamped();p.pose.position.x=float(data['x']);p.pose.position.y=float(data['y']);p.pose.orientation.w=1.
            return p
        r.pose_message=pose

    def test_segment_uses_explicit_start_without_drive(self):
        self.r.send_goal(dict(x=2,y=3),'A',preview=True,start=dict(x=1,y=1))
        goal=self.r.planner.send_goal_async.call_args.args[0]
        self.assertTrue(goal.use_start)
        self.assertEqual(goal.start.pose.position.x,1.)
        self.r.send_drive.assert_not_called()

    def test_late_acceptance_after_cancel_is_cancelled(self):
        done=Mock()
        self.r.send_goal(dict(x=2,y=3),'A',preview=True,on_result=done)
        self.r.cancel();handle=Mock();handle.accepted=True
        self.future.resolve(handle)
        handle.cancel_goal_async.assert_called_once();done.assert_not_called()

    def test_success_callback_gets_path_without_motion(self):
        done=Mock();self.r.send_goal(dict(x=2,y=3),'A',preview=True,on_result=done)
        result=Future();handle=Mock();handle.accepted=True;handle.get_result_async.return_value=result
        self.future.resolve(handle)
        poses=[self.r.pose_message(dict(x=0,y=0)),self.r.pose_message(dict(x=2,y=3))]
        result.resolve(S(status=4,result=S(path=S(poses=poses))))
        done.assert_called_once_with([[0.,0.],[2.,3.]],None)
        self.r.send_drive.assert_not_called()

    def test_failed_result_propagates_to_route(self):
        done=Mock();self.r.send_goal(dict(x=2,y=3),'A',preview=True,on_result=done)
        result=Future();handle=Mock();handle.accepted=True;handle.get_result_async.return_value=result
        self.future.resolve(handle);result.resolve(S(status=6))
        self.assertIsNone(done.call_args.args[0]);self.assertTrue(done.call_args.args[1])
        self.r.send_drive.assert_not_called()

    def test_map_mismatch_rejects_before_planning(self):
        data=task();data['map']='different'
        with self.assertRaises(ValueError):self.r.preview_patrol(data,'A')
        self.r.planner.send_goal_async.assert_not_called()

    def test_send_failure_does_not_leave_pending_task(self):
        self.r.planner.send_goal_async.side_effect=RuntimeError('offline')
        done=Mock()
        self.r.send_goal(dict(x=2,y=3),'A',preview=True,on_result=done)
        self.assertFalse(self.r.pending)
        done.assert_called_once_with(None,'offline')


if __name__=='__main__':unittest.main()
