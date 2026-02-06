#!/bin/bash
# Simplest possible check - just ping robot and verify intera.sh exists

ROBOT_IP="192.168.1.103"

# Try ping
if ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
    echo "✓ Robot reachable at $ROBOT_IP"

    # Check intera.sh exists
    if [ -f ~/ros_ws/intera.sh ]; then
        echo "✓ intera.sh found"
        echo ""
        echo "Ready! Run: cd ~/ros_ws && ./intera.sh"
        exit 0
    else
        echo "✗ intera.sh not found"
        exit 1
    fi
else
    echo "✗ Cannot reach robot at $ROBOT_IP"
    exit 1
fi
