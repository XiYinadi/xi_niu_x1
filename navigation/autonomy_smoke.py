"""Board integration acceptance. Never arms or sends motion commands."""
import json
import sys
import time
import urllib.request
sys.path.extend(['/usr/local/lib/python3.10/dist-packages','/usr/lib/python3/dist-packages'])
import socketio

state={};errors=[]
client=socketio.Client()
client.on('navigation_state',lambda d:state.update(d))
client.on('navigation_error',lambda d:errors.append(d['message']))
client.connect('http://127.0.0.1:5800',transports=['polling'])
def wait(test,seconds=55):
    end=time.monotonic()+seconds;last=0
    while time.monotonic()<end:
        if test():return
        if time.monotonic()-last>10:
            print(json.dumps({k:state.get(k) for k in ('status','running','mode','localized','planner_ready','navigator_ready','message')},ensure_ascii=False),flush=True)
            last=time.monotonic()
        time.sleep(.2)
    raise RuntimeError('timed out: '+json.dumps(state,ensure_ascii=False)+' errors='+str(errors))
wait(lambda:'mode' in state)
if not state['running']:
    client.emit('navigation_control',dict(action='prepare_mapping'))
wait(lambda:state.get('mode')=='mapping' and state.get('localized') and state.get('navigator_ready') and state.get('planner_ready'))
print('MAPPING_NAV2_READY',flush=True)
client.emit('navigation_control',dict(action='explore_preview'))
wait(lambda:state.get('autonomy',{}).get('preview') is not None,20)
print('EXPLORATION_PREVIEW',json.dumps(state['autonomy']['preview'],ensure_ascii=False),flush=True)
client.emit('navigation_control',dict(action='explore_start',name='reject_uncalibrated'))
wait(lambda:bool(errors),5)
assert '标定' in errors[-1],errors
with urllib.request.urlopen('http://127.0.0.1:5800/api/autonomy/state') as r:autonomy=json.load(r)
assert not autonomy['active']
print('UNCALIBRATED_START_REJECTED',errors[-1],flush=True)
print('PASS: live mapping/navigation data; read-only frontier preview; motion stays disabled',flush=True)
client.disconnect()
