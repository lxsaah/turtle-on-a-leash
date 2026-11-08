#!/bin/bash
# turtlesim on a virtual display, served to the browser on port 6080.
set -e
Xvfb :1 -screen 0 520x520x24 &
sleep 1
x11vnc -display :1 -nopw -forever -shared -rfbport 5900 -bg -quiet
websockify --web /usr/share/novnc 6080 localhost:5900 &
export DISPLAY=:1
exec ros2 run turtlesim turtlesim_node
