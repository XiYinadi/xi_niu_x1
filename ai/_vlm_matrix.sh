#!/bin/bash
# 诊断矩阵:每用例重启服务 -> 单次请求 -> 看存活
run_test() {
  echo ""
  echo "===== $1 ====="
  echo "aidlux" | sudo -S systemctl restart aidllm-api.service 2>/dev/null
  for i in $(seq 1 40); do
    ok=$(curl -s -m 2 http://127.0.0.1:8888/v1/models 2>/dev/null | grep -c qwen2.5-vl)
    [ "$ok" -ge 1 ] && break
    sleep 1
  done
  shift
  /usr/bin/python3 ~/_vlm_req.py "$@"
  sleep 3
  curl -s -m 3 http://127.0.0.1:8888/v1/models >/dev/null 2>&1 && echo "  -> 服务存活" || echo "  -> 服务崩了"
}

run_test "A: SHORT prompt + 小图(对照)" ~/test_red_circle.jpg short
run_test "B: LONG prompt  + 小图" ~/test_red_circle.jpg long
run_test "C: SHORT prompt + 大相机图" CAM short
run_test "D: LONG prompt  + 大相机图" CAM long
