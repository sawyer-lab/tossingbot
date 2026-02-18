#!/bin/bash
# Wrapper script to start ZMQ server with proper ROS environment

# Source ROS environment (required for intera_core_msgs and other ROS packages)
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

# Apply runtime IPs to intera.sh (replaces build-time hardcoded values) then
# source it.  init_robot.sh is a no-op when HOST_IP / ROBOT_IP are not set.
source ~/init_robot.sh

# Explicit overrides so ROS_MASTER_URI and ROS_IP are always correct
# regardless of what values intera.sh baked in.
#
# ROS_IP must be the HOST's IP (not the robot's IP) so the robot can connect
# back to this machine to complete publisher/subscriber handshakes.
# (Skipped in sim mode where ROBOT_IP/HOST_IP are not set.)
if [ -n "${ROBOT_IP}" ] && [ -n "${HOST_IP}" ]; then
    export ROS_MASTER_URI=http://${ROBOT_IP}:11311
    export ROS_IP=${HOST_IP}
fi

# Run the ZMQ server
cd ~/ros_ws/src/custom/robot_api/servers
exec python3 zmq_server.py "$@"
