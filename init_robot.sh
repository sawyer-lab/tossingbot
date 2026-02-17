#!/bin/bash

INTERA_PATH=~/ros_ws/intera.sh

if [ -z "$HOST_IP" ] || [ -z "$ROBOT_IP" ]; then
    return 0 2>/dev/null || exit 0
fi

if [ -f "$INTERA_PATH" ]; then
    sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" "$INTERA_PATH"
    sed -i "s/robot_hostname=\".*\"/robot_hostname=\"${ROBOT_HOSTNAME:-$ROBOT_IP}\"/g" "$INTERA_PATH"
    echo "Robot configured: HOST_IP=$HOST_IP ROBOT_IP=${ROBOT_HOSTNAME:-$ROBOT_IP}"
fi
