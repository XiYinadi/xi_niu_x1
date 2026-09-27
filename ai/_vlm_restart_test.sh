#!/bin/bash
# 诊断:无请求时 VLM 服务是否稳定存活
echo "=== 停掉 fire_detect ==="
pkill -f fire_detect.py 2>/dev/null
sleep 1
ps aux | grep fire_detect | grep -v grep && echo "fire_detect 还在" || echo "fire_detect 已停"

echo "=== 重启 VLM 服务 ==="
echo "aidlux" | sudo -S systemctl restart aidllm-api.service 2>/dev/null

echo "等模型就绪..."
for i in $(seq 1 40); do
  ok=$(curl -s -m 2 http://127.0.0.1:8888/v1/models 2>/dev/null | grep -c qwen2.5-vl)
  if [ "$ok" -ge 1 ]; then echo "就绪于第 ${i}s"; break; fi
  sleep 1
done

echo "=== 无请求空跑 20s,看服务是否存活 ==="
sleep 20
curl -s -m 3 http://127.0.0.1:8888/v1/models >/dev/null 2>&1 && echo "OK 20s后仍存活" || echo "DEAD 20s内自己死了"
aidllm status api 2>&1 | head -2
