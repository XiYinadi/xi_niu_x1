#!/bin/bash
source /opt/ros/humble/setup.bash
# Ensure Xvfb is running
pgrep Xvfb > /dev/null || (echo aidlux | sudo -S Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render &>/dev/null & sleep 1)
# Ensure x11vnc is running
pgrep x11vnc > /dev/null || (x11vnc -display :99 -forever -shared -passwd aidlux -rfbport 5900 &>/dev/null & sleep 1)
# Kill old rviz2 and start new
pkill rviz2 2>/dev/null
DISPLAY=:99 rviz2 -d /tmp/view_lidar.rviz
