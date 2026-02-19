#!/bin/bash

INTERA_PATH=~/ros_ws/intera.sh

# If we have runtime IPs, patch intera.sh
if [ -n "$HOST_IP" ] && [ -n "$ROBOT_IP" ]; then
    if [ -f "$INTERA_PATH" ]; then
        # Always use IP (not hostname) so ROS_MASTER_URI resolves inside the container
        sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" "$INTERA_PATH"
        sed -i "s/robot_hostname=\".*\"/robot_hostname=\"${ROBOT_IP}\"/g" "$INTERA_PATH"
        echo "Robot configured at runtime: HOST_IP=$HOST_IP ROBOT_IP=$ROBOT_IP"
    fi
fi

# Source intera.sh if it exists (always, to set environment)
if [ -f "$INTERA_PATH" ]; then
    source "$INTERA_PATH"
fi
