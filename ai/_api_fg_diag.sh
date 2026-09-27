#!/bin/bash
# 前台诊断:api 进程 stderr 落盘 -> 打一次请求 -> 抓崩溃现场
cd /opt/aidlux/app/aid-openai-api || exit 1

echo "=== 模型文件检查 ==="
ls -d ./models/qwen2.5-vl-3b-instruct-392x392-qnn2.36-w4a16-qcs8550 2>/dev/null || echo "模型目录缺失!"
ls ./models/qwen2.5-vl-3b-instruct-392x392-qnn2.36-w4a16-qcs8550/*.json 2>/dev/null | head -2

echo "=== 启动(前台,stderr落盘) ==="
setsid nohup bash -c 'cd /opt/aidlux/app/aid-openai-api && LD_LIBRARY_PATH=./lib ./api --device dsp --qnn_ver 236 --config ./api_cfg.json > /tmp/api_fg.log 2>&1' </dev/null >/dev/null 2>&1 &
echo "已启动 PID=$!"

echo "=== 等就绪 ==="
for i in $(seq 1 40); do
  ok=$(curl -s -m 2 http://127.0.0.1:8888/v1/models 2>/dev/null | grep -c qwen2.5-vl)
  [ "$ok" -ge 1 ] && { echo "就绪于第 ${i}s"; break; }
  sleep 1
done

echo "=== 启动日志前 20 行 ==="
head -20 /tmp/api_fg.log

echo "=== 打控制请求 ==="
/usr/bin/python3 ~/_vlm_req.py ~/test_red_circle.jpg short

echo "=== 请求后等待 4s ==="
sleep 4
pgrep -af 'api --device' >/dev/null && echo "进程仍存活" || echo ">>> 进程已死 <<<"

echo "=== stderr 日志尾部 40 行(崩溃现场) ==="
tail -40 /tmp/api_fg.log
