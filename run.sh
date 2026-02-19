#!/bin/bash
# Start the robot container with automatic network detection and firewall setup

set -e

xhost +local:root 2>/dev/null || true

echo "Searching for Sawyer robot on the network …"
eval "$(./find_robot.sh)"

# Allow inbound connections from the robot (needed for ROS publisher handshake)
# Uses -n so it never prompts — run once manually if needed: sudo iptables -I INPUT -s <ROBOT_IP> -j ACCEPT
if ! sudo -n iptables -C INPUT -s "$ROBOT_IP" -j ACCEPT 2>/dev/null; then
    sudo -n iptables -I INPUT -s "$ROBOT_IP" -j ACCEPT 2>/dev/null || \
        echo "  ⚠ Could not add firewall rule (run: sudo iptables -I INPUT -s $ROBOT_IP -j ACCEPT)"
fi

echo ""
echo "Starting robo2025_dev container (physical robot)..."
echo "  ROBOT_IP : $ROBOT_IP"
echo "  HOST_IP  : $HOST_IP"

ROBOT_IP=$ROBOT_IP HOST_IP=$HOST_IP \
    docker-compose up -d --force-recreate robot_dev

echo ""
echo "✓ Robot container started!"
echo "  Enter: ./exec.sh"
