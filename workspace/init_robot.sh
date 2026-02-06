#!/bin/bash
# Runtime configuration script for container
# Patches intera.sh with environment variables passed from host

INTERA_PATH=~/ros_ws/intera.sh

# Check if environment variables are set
if [ -z "$HOST_IP" ] || [ -z "$ROBOT_IP" ]; then
    # No robot env vars - container started with run.sh (simulator mode)
    return 0 2>/dev/null || exit 0
fi

# Patch intera.sh with runtime values
if [ -f "$INTERA_PATH" ]; then
    sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" "$INTERA_PATH"
    sed -i "s/robot_hostname=\".*\"/robot_hostname=\"${ROBOT_HOSTNAME:-$ROBOT_IP}\"/g" "$INTERA_PATH"

    echo "✓ intera.sh configured with runtime values:"
    echo "  your_ip=$HOST_IP"
    echo "  robot_hostname=${ROBOT_HOSTNAME:-$ROBOT_IP}"
fi
