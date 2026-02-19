#!/bin/bash
# Build ROS workspace inside running container
# Run this ONCE after first container start to compile custom packages

set -e

echo "=========================================="
echo "Building ROS Workspace"
echo "=========================================="
echo ""

# Source ROS
source /opt/ros/noetic/setup.bash

# Go to workspace
cd ~/ros_ws

echo "Running catkin_make..."
catkin_make

echo ""
echo "✓ Workspace built successfully!"
echo ""
echo "To use the robot, run:"
echo "  source ~/ros_ws/devel/setup.bash"
echo "  source ~/ros_ws/intera.sh"
echo "  cd ~/ros_ws/src/custom/robot_api/servers"
echo "  python3 zmq_server.py"
