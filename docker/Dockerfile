# ROS 2 Jazzy with rmw_zenoh, turtlesim, and a browser view of its window.
FROM ros:jazzy
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-zenoh-cpp ros-jazzy-geometry-msgs ros-jazzy-turtlesim \
      xvfb x11vnc novnc websockify imagemagick \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
COPY start-turtlesim.sh /start-turtlesim.sh
RUN chmod +x /start-turtlesim.sh
