#!/bin/bash
set -e

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash

# Setup local workspace environment if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
