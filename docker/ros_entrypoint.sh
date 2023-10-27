#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" && \
source "$RBK_CATKIN_PATH/devel/setup.bash" && \
exec "$@"