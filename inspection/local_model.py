"""Local VLM HTTP access, serialized across camera and report processes."""
import contextlib
import json
import time
import urllib.request


@contextlib.contextmanager
def inference_slot(timeout=80):
    import fcntl
    deadline = time.monotonic()+timeout
    with open('/tmp/robot_local_vlm.lock', 'a') as file:
        while True:
            try:
                fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise TimeoutError('本地模型繁忙，请稍后重试')
                time.sleep(.1)
        try:
            yield
        finally:
            fcntl.flock(file, fcntl.LOCK_UN)


def completion(payload, api='http://127.0.0.1:8888', timeout=80):
    req=urllib.request.Request(api+'/v1/chat/completions',
        data=json.dumps(payload).encode(), headers={'Content-Type':'application/json'})
    with inference_slot(timeout):
        with urllib.request.urlopen(req, timeout=timeout) as response:
            data=json.load(response)
    return data['choices'][0]['message']['content']


def loaded_model():
    with urllib.request.urlopen('http://127.0.0.1:8888/v1/models', timeout=3) as response:
        data=json.load(response)
    model=data.get('loaded_id') or (data.get('data') or [{}])[0].get('id')
    if not model:
        raise ValueError('本地模型尚未加载')
    return model
