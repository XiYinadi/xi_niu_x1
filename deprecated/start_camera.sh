#!/bin/bash
LOG=~/camera_preview.log
SCRIPT=~/camera_preview.py

OLD=$(ps aux | grep camera_preview.py | grep -v grep | awk "{print \$2}")
if [ -n "$OLD" ]; then
    kill $OLD 2>/dev/null
    sleep 1
    echo "stopped old: $OLD"
fi

export DISPLAY=:2
nohup python3 -u $SCRIPT > $LOG 2>&1 &
PID=$!
sleep 4

if grep -q "摄像头已连接" $LOG 2>/dev/null; then
    echo "OK - camera preview running (PID: $PID)"
    echo "VNC: 192.168.31.218:5902"
else
    tail -3 $LOG
fi
