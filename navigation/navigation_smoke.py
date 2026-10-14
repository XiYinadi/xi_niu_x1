"""On-board non-driving acceptance: save, reload, localize, plan, reject motion."""
import json
import math
import sys
import time
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import socketio
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

state = {}
errors = []
client = socketio.Client()
client.on('navigation_state', lambda data: state.update(data))
client.on('navigation_error', lambda data: errors.append(data['message']))


def wait(predicate, seconds=30):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate(): return
        time.sleep(.2)
    raise RuntimeError('timeout: '+json.dumps(state,ensure_ascii=False)+' errors='+repr(errors))


client.connect('http://127.0.0.1:5800', transports=['polling'])
wait(lambda: 'maps' in state)
if '--reload' in sys.argv:
    from pathlib import Path
    recorded=json.loads(Path('/tmp/navigation_acceptance.json').read_text())
    pose=recorded['initial_pose'];name=recorded['map']
else:
    pose=None
rclpy.init()
node = rclpy.create_node('navigation_acceptance_pose')
buffer = Buffer(); listener = TransformListener(buffer,node)
for _ in range(0 if pose else 100):
    rclpy.spin_once(node,timeout_sec=.1)
    try:
        tf=buffer.lookup_transform('map','base_link',Time()).transform
        q=tf.rotation
        pose=dict(x=tf.translation.x,y=tf.translation.y,
                  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        break
    except Exception: pass
node.destroy_node();rclpy.shutdown()
if pose is None: raise RuntimeError('Cannot establish actual current pose before reload')
if '--reload' not in sys.argv:
    name='acceptance_'+time.strftime('%Y%m%d_%H%M%S')
    client.emit('navigation_control',dict(action='save',name=name))
    wait(lambda: name in state.get('maps',[]))
print('Saved map:',name,'current map pose:',pose,flush=True)
client.emit('navigation_control',dict(action='load',name=name))
wait(lambda: state.get('running'),45)
wait(lambda: state.get('initial_pose_ready'),30)
client.emit('navigation_control',dict(action='initial_pose',**pose))
wait(lambda: state.get('localized'),30)
wait(lambda: state.get('planner_ready') and state.get('navigator_ready'),45)
print('Nav2 lifecycle states ACTIVE',flush=True)
print('Reload localization ready:',state.get('pose'),flush=True)
from pathlib import Path
Path('/tmp/navigation_acceptance.json').write_text(json.dumps(dict(map=name,initial_pose=pose,state=state),ensure_ascii=False))
# Goal at the current location exercises the actual planner without requiring a
# particular free corridor or asserting that unknown surroundings are traversable.
client.emit('navigation_control',dict(action='preview',**pose))
wait(lambda: bool(state.get('path')) or '未完成' in state.get('message',''),30)
print('Planner result:',len(state['path']),'poses;',state.get('message'),flush=True)
errors.clear()
client.emit('navigation_control',dict(action='goal',**pose))
wait(lambda: bool(errors),5)
assert not state.get('navigating'),state
print('Uncalibrated motion correctly rejected:',errors[-1],flush=True)
client.disconnect()
