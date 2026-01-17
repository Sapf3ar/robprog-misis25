#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

# Source custom workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

exec "$@"
