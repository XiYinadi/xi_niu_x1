#!/bin/bash
echo aidlux | sudo -S Xvfb :98 -screen 0 1280x720x24 -ac +extension GLX +render &>/dev/null &
sleep 2
x11vnc -display :98 -forever -shared -nopw -rfbport 5902 &>/dev/null &
sleep 1
DISPLAY=:98 dbus-launch --exit-with-session gnome-session --session=ubuntu &>/dev/null &
sleep 4
echo 'VNC ready on port 5902'
