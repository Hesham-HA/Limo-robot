#!/bin/bash

set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source Gazebo setup
source /usr/share/gazebo/setup.bash

# If a catkin workspace exists, source it
if [ -f /home/ros-noetic/catkin_ws/devel/setup.bash ]; then
    source /home/ros-noetic/catkin_ws/devel/setup.bash
fi

# Execute passed commands or keep shell open
exec "$@"