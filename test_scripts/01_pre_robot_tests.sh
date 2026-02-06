#!/bin/bash
# Run BEFORE turning on the robot
# Tests container network configuration

echo "=========================================="
echo "PRE-ROBOT BOOT TESTS"
echo "=========================================="
echo ""

echo "1. Testing basic network tools availability..."
command -v ping >/dev/null 2>&1 && echo "  ✓ ping available" || echo "  ✗ ping NOT available"
command -v avahi-resolve >/dev/null 2>&1 && echo "  ✓ avahi-resolve available" || echo "  ✗ avahi-resolve NOT available"
command -v avahi-browse >/dev/null 2>&1 && echo "  ✓ avahi-browse available" || echo "  ✗ avahi-browse NOT available"
command -v ip >/dev/null 2>&1 && echo "  ✓ ip available" || echo "  ✗ ip NOT available"
echo ""

echo "2. Checking network interfaces (from container perspective)..."
ip -br addr show
echo ""

echo "3. Checking if we can see host ethernet interface..."
if ip addr show enp0s13f0u4c2 2>/dev/null; then
    echo "  ✓ Can see enp0s13f0u4c2"
else
    echo "  ✗ Cannot see enp0s13f0u4c2 (expected with --network host)"
fi
echo ""

echo "4. Testing avahi-daemon status..."
if pgrep -x "avahi-daemon" > /dev/null; then
    echo "  ✓ avahi-daemon is running"
else
    echo "  ✗ avahi-daemon is NOT running"
    echo "  Attempting to start avahi-daemon..."
    sudo service avahi-daemon start 2>/dev/null || echo "  Cannot start (may need to run as root or add to Dockerfile)"
fi
echo ""

echo "5. Checking current intera.sh configuration..."
if [ -f ~/ros_ws/intera.sh ]; then
    echo "  Robot hostname configured as:"
    grep "robot_hostname=" ~/ros_ws/intera.sh | head -1
    echo "  Your IP configured as:"
    grep "your_ip=" ~/ros_ws/intera.sh | head -1
    echo "  ROS version configured as:"
    grep "ros_version=" ~/ros_ws/intera.sh | head -1
else
    echo "  ✗ intera.sh not found at ~/ros_ws/intera.sh"
fi
echo ""

echo "6. Testing connectivity to subnet gateway..."
ping -c 1 -W 1 192.168.1.1 >/dev/null 2>&1 && echo "  ✓ Can reach 192.168.1.1 (gateway)" || echo "  ✗ Cannot reach gateway"
echo ""

echo "7. Current avahi services (should be empty before robot boots)..."
timeout 3 avahi-browse -apt 2>/dev/null | grep -i rethink || echo "  (No Rethink services found - expected before robot boots)"
echo ""

echo "=========================================="
echo "PRE-ROBOT TESTS COMPLETE"
echo "You can now turn on the robot"
echo "=========================================="
