import copy
import math
import tempfile
import unittest
from pathlib import Path
from patrol_store import PatrolStore, validate_task
from patrol_preview import PatrolPreview


def task():
    return dict(name='round_a', map='room', cycles=2, revision=0,
        waypoints=[dict(x=1,y=2,yaw=0,dwell_s=5),dict(x=3,y=4,yaw=1,dwell_s=10)])


class Tasks(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        class Maps:
            def path(self, name):
                if name!='room': raise ValueError('missing map')
        self.store=PatrolStore(self.temp.name,Maps())

    def tearDown(self): self.temp.cleanup()

    def test_roundtrip_survives_store_restart(self):
        saved=self.store.save(task())
        again=PatrolStore(self.temp.name,self.store.maps)
        self.assertEqual(again.get('round_a'),saved)
        self.assertEqual(again.list()[0]['points'],2)

    def test_stale_client_cannot_overwrite(self):
        original=self.store.save(task())
        client=copy.deepcopy(original);client['cycles']=3
        self.store.save(client)
        with self.assertRaises(ValueError): self.store.save(original)
        self.assertEqual(self.store.get('round_a')['cycles'],3)

    def test_missing_map_rejected(self):
        data=task();data['map']='absent'
        with self.assertRaises(ValueError):self.store.save(data)

    def test_path_escape_rejected(self):
        data=task();data['name']='../escape'
        with self.assertRaises(ValueError):self.store.save(data)

    def test_empty_and_large_task_rejected(self):
        for points in ([],task()['waypoints']*26):
            data=task();data['waypoints']=points
            with self.assertRaises(ValueError):self.store.save(data)

    def test_bad_numbers_rejected(self):
        for key,value in [('x',float('nan')),('yaw',float('inf')),('y','1'),('dwell_s',-1),('dwell_s',True)]:
            data=task();data['waypoints'][0][key]=value
            with self.assertRaises(ValueError):validate_task(data)

    def test_cycles_are_bounded_integers(self):
        for value in (0,101,1.5,True,'2'):
            data=task();data['cycles']=value
            with self.assertRaises(ValueError):validate_task(data)

    def test_save_does_not_mutate_draft(self):
        data=task();self.store.save(data)
        self.assertEqual(data['revision'],0)


class Preview(unittest.TestCase):
    def setUp(self):
        self.preview=PatrolPreview();self.calls=[]
        self.plan=lambda start,target,done:self.calls.append((start,target,done))
        self.origin=dict(x=0,y=0,yaw=0)

    def test_segments_include_loop_closure_and_dwell_total(self):
        self.preview.start(task(),self.origin,self.plan)
        self.calls[0][2]([[0,0],[1,2]],None)
        self.calls[1][2]([[1,2],[3,4]],None)
        self.calls[2][2]([[3,4],[1,2]],None)
        state=self.preview.snapshot()
        self.assertEqual(state['status'],'ready')
        self.assertEqual(state['path'],[[0,0],[1,2],[3,4],[1,2]])
        self.assertEqual(state['planned_stopped_s'],30)

    def test_single_cycle_does_not_add_return(self):
        data=task();data['cycles']=1
        self.preview.start(data,self.origin,self.plan)
        self.assertEqual(self.preview.state['total'],2)

    def test_failure_never_claims_complete(self):
        self.preview.start(task(),self.origin,self.plan)
        self.calls[0][2]([[0,0],[1,2]],None)
        self.calls[1][2](None,'obstacle')
        self.assertEqual(self.preview.state['status'],'failed')
        self.assertEqual(self.preview.state['segment'],2)
        self.assertEqual(len(self.calls),2)

    def test_empty_path_is_failure(self):
        self.preview.start(task(),self.origin,self.plan)
        self.calls[0][2]([],None)
        self.assertEqual(self.preview.state['status'],'failed')

    def test_cancel_ignores_late_result(self):
        self.preview.start(task(),self.origin,self.plan)
        self.preview.cancel()
        self.calls[0][2]([[0,0],[1,2]],None)
        self.assertEqual(len(self.calls),1)
        self.assertEqual(self.preview.state['status'],'cancelled')

    def test_old_task_cannot_pollute_new_task(self):
        self.preview.start(task(),self.origin,self.plan)
        old=self.calls[0][2];self.preview.cancel()
        self.preview.start(task(),self.origin,self.plan)
        old([[0,0],[99,99]],None)
        self.assertEqual(self.preview.state['path'],[])


if __name__=='__main__':unittest.main()
