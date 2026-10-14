import base64
import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from flask import Flask
from inspection_service import InspectionService,statistics,register_inspection,summarize,classify


def sample(after):
    return dict(fire=False,confidence=.9,reason='未见明火',captured_at=after+.001,
        judged_at=time.time(),model='local-test',jpeg=base64.b64encode(b'\xff\xd8test\xff\xd9').decode(),conf_thresh=.5)


class Inspection(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.task=dict(name='route',map='room',cycles=1,waypoints=[dict(x=1,y=2,yaw=0,dwell_s=5)])
        self.service=InspectionService(self.root,task_get=lambda name:copy.deepcopy(self.task),sample=sample,
            analyze=lambda s,p:dict(model='test',summary='事实摘要',recommendations=[]))
        self.sid=self.service.start({})['id']

    def tearDown(self):self.temp.cleanup()

    def wait(self,predicate):
        until=time.monotonic()+3
        while time.monotonic()<until:
            with self.service.lock:
                if predicate():return
            time.sleep(.01)
        self.fail('Background operation did not finish')

    def recorded(self):
        self.service.record(self.sid)
        self.wait(lambda:self.service.get(self.sid)['status']=='running')
        return self.service.get(self.sid)

    def test_evidence_is_saved_and_hashed(self):
        s=self.recorded();r=s['records'][0]
        self.assertEqual(r['status'],'ok');self.assertEqual(r['verdict'],'clear')
        self.assertTrue((self.service.directory(self.sid)/r['image']).is_file())
        self.assertEqual(len(r['sha256']),64)

    def test_old_frame_rejected_and_does_not_advance(self):
        self.service.sample=lambda after:dict(sample(after),captured_at=after-10)
        s=self.recorded()
        self.assertEqual(s['next_step'],0);self.assertEqual(s['records'][0]['status'],'error')
        self.assertEqual(statistics(s)['checked'],0)

    def test_string_false_is_not_accepted_as_boolean(self):
        self.service.sample=lambda after:dict(sample(after),fire='false')
        self.assertEqual(self.recorded()['records'][0]['status'],'error')

    def test_low_confidence_is_uncertain_not_clear(self):
        self.service.sample=lambda after:dict(sample(after),confidence=.2)
        self.assertEqual(statistics(self.recorded())['uncertain'],1)

    def test_suspected_fire_is_not_summarized_clear(self):
        self.service.sample=lambda after:dict(sample(after),fire=True,reason='物体正在燃烧')
        counts=statistics(self.recorded())
        self.assertEqual(counts['suspected'],1);self.assertIn('疑似',counts['overall'])

    def test_contradictory_model_explanation_is_uncertain(self):
        for reason in ('明显的烟柱','没有明火，但有烟柱'):
            self.assertEqual(classify(dict(fire=False,confidence=.95,reason=reason)),'uncertain')

    def test_negative_explanation_is_not_a_false_alarm(self):
        self.assertEqual(classify(dict(fire=False,confidence=.95,reason='未见明火或烟柱')),'clear')
        self.assertEqual(classify(dict(fire=False,confidence=.95,reason='明灯反射误判为烟柱')),'clear')

    def test_failed_detection_can_retry_same_step(self):
        self.service.sample=lambda after:(_ for _ in ()).throw(TimeoutError('offline'))
        self.recorded();self.service.sample=sample
        s=self.recorded();self.assertEqual([r['step'] for r in s['records']],[0,0])
        self.assertEqual(statistics(s)['checked'],1)

    def test_last_task_point_auto_generates_report(self):
        self.service.get(self.sid)['status']='completed'
        self.sid=self.service.start(dict(task='route'))['id']
        self.service.record(self.sid)
        self.wait(lambda:self.service.get(self.sid)['status']=='completed')
        self.assertEqual(self.service.get(self.sid)['report_status'],'ready')

    def test_skip_is_missing_not_checked(self):
        self.service.get(self.sid)['status']='completed'
        self.sid=self.service.start(dict(task='route'))['id']
        self.service.record(self.sid,skip=True)
        self.wait(lambda:self.service.get(self.sid)['status']=='completed')
        counts=statistics(self.service.get(self.sid))
        self.assertEqual(counts['missing'],1);self.assertEqual(counts['checked'],0)

    def test_model_failure_preserves_evidence_report(self):
        self.recorded();self.service.analyze=lambda s,p:(_ for _ in ()).throw(RuntimeError('model offline'))
        self.service.finish(self.sid)
        self.wait(lambda:self.service.get(self.sid)['status']=='completed')
        s=self.service.get(self.sid)
        self.assertEqual(s['report_status'],'failed');self.assertEqual(len(s['records']),1)

    def test_restart_marks_inflight_operation_interrupted(self):
        s=self.service.get(self.sid);s['status']='sampling';self.service.persist(s)
        restored=InspectionService(self.root)
        self.assertEqual(restored.get(self.sid)['status'],'running')
        self.assertEqual(restored.get(self.sid)['report_status'],'interrupted')

    def test_second_session_rejected_while_active(self):
        with self.assertRaises(ValueError):self.service.start({})

    def test_report_model_cannot_change_actual_counts(self):
        s=self.recorded()
        raw=json.dumps(dict(overall='everything safe',checked=999,summary='wrong',recommendations=[]))
        with patch('inspection_service.loaded_model',return_value='test'),patch('inspection_service.completion',return_value=raw):
            with self.assertRaises(ValueError):summarize(s,self.service.directory(self.sid))

    def test_route_coverage_includes_unvisited_points(self):
        s=self.recorded();s['planned_steps']=5
        self.assertEqual(statistics(s)['missing'],4)

    def test_report_rejects_broad_safety_claim(self):
        s=self.recorded();counts=statistics(s)
        raw=json.dumps(dict(overall=counts['overall'],checked=counts['checked'],summary='整体环境安全且无危险',recommendations=[]))
        with patch('inspection_service.loaded_model',return_value='test'),patch('inspection_service.completion',return_value=raw):
            with self.assertRaises(ValueError):summarize(s,self.service.directory(self.sid))

    def test_report_prompt_supplies_literal_statistics(self):
        s=self.recorded();counts=statistics(s)
        raw=json.dumps(dict(overall=counts['overall'],checked=counts['checked'],summary='已检测1点，未见明显火焰烟雾。',recommendations=[]))
        with patch('inspection_service.loaded_model',return_value='test'),patch('inspection_service.completion',return_value=raw) as request:
            self.assertEqual(summarize(s,self.service.directory(self.sid))['summary'],'已检测1点，未见明显火焰烟雾。')
            prompt=request.call_args.args[0]['messages'][0]['content'][0]['text']
            self.assertIn(json.dumps(dict(overall=counts['overall'],checked=counts['checked']),ensure_ascii=False)[1:-1],prompt)

    def test_report_removes_unsupported_sentence_preserving_original(self):
        s=self.recorded();counts=statistics(s)
        raw=json.dumps(dict(overall=counts['overall'],checked=counts['checked'],summary='未发现明显火焰或烟雾。整体环境安全且无危险。',recommendations=[]))
        with patch('inspection_service.loaded_model',return_value='test'),patch('inspection_service.completion',return_value=raw):
            report=summarize(s,self.service.directory(self.sid))
            self.assertEqual(report['summary'],'未发现明显火焰或烟雾。')
            self.assertTrue(report['review_note'])
            self.assertEqual(json.loads((self.service.directory(self.sid)/'model_response.json').read_text())['content'],raw)

    def test_automatic_session_rejects_manual_detection(self):
        s=self.service.get(self.sid);s['mode']='automatic'
        with self.assertRaises(ValueError):self.service.record(self.sid)
        self.assertEqual(s['records'],[])

    def test_alert_changes_overall_without_claiming_point_arrival(self):
        s=self.service.get(self.sid);s['alerts']=[dict(verdict='suspected')]
        stats=statistics(s)
        self.assertEqual(stats['checked'],0);self.assertEqual(stats['suspected'],1)
        self.assertIn('疑似',stats['overall'])

    def test_api_report_escapes_untrusted_text(self):
        s=self.recorded();s['title']='<script>alert(1)</script>'
        self.service.finish(self.sid);self.wait(lambda:s['status']=='completed')
        app=Flask(__name__)
        register_inspection(app,self.service,Path(__file__).resolve().parent.parent/'web')
        client=app.test_client()
        response=client.get('/inspection/reports/'+self.sid)
        self.assertEqual(response.status_code,200)
        self.assertIn(b'&lt;script&gt;',response.data)
        self.assertNotIn(b'<script>alert(1)</script>',response.data)
        self.assertEqual(client.get('/inspection/evidence/'+self.sid+'/not-an-image').status_code,400)


if __name__=='__main__':unittest.main()
