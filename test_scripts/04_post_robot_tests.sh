#!/bin/bash
# Run AFTER robot is detected online
# Comprehensive connectivity and ROS tests

ROBOT_HOSTNAME="021607CP00070.local"
ROBOT_IP="192.168.1.101"

echo "=========================================="
echo "POST-ROBOT BOOT TESTS"
echo "=========================================="
echo ""

echo "1. Network connectivity tests..."
echo "  Testing ping to IP..."
if ping -c 3 $ROBOT_IP >/dev/null 2>&1; then
    latency=$(ping -c 3 $ROBOT_IP 2>/dev/null | tail -1 | awk -F'/' '{print $5}')
    echo "    ✓ Ping to $ROBOT_IP successful (avg ${latency}ms)"
else
    echo "    ✗ Ping to $ROBOT_IP FAILED"
fi

echo "  Testing ping to hostname..."
if ping -c 3 $ROBOT_HOSTNAME >/dev/null 2>&1; then
    latency=$(ping -c 3 $ROBOT_HOSTNAME 2>/dev/null | tail -1 | awk -F'/' '{print $5}')
    echo "    ✓ Ping to $ROBOT_HOSTNAME successful (avg ${latency}ms)"
else
    echo "    ✗ Ping to $ROBOT_HOSTNAME FAILED"
fi
echo ""

echo "2. mDNS resolution tests..."
echo "  Testing avahi-resolve..."
resolved=$(avahi-resolve -4 -n $ROBOT_HOSTNAME 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "    ✓ $resolved"
else
    echo "    ✗ avahi-resolve failed"
fi

echo "  Testing avahi-browse for Rethink services..."
timeout 5 avahi-browse -pt _rethink-robot._tcp 2>/dev/null | grep "IPv4" | head -3 | while read line; do
    echo "    Found: $line"
done
echo ""

echo "3. Checking ROS ports (robot side)..."
echo "  Testing port 11311 (ROS Master)..."
timeout 2 nc -zv $ROBOT_IP 11311 2>&1 | grep -q succeeded && echo "    ✓ Port 11311 is open" || echo "    ✗ Port 11311 is closed/filtered"

echo "  Testing if robot is in SDK mode vs Research mode..."
if timeout 2 nc -zv $ROBOT_IP 11311 2>&1 | grep -q succeeded; then
    echo "    ℹ️  Robot appears to be in RESEARCH mode (has ROS master)"
else
    echo "    ℹ️  Robot might be in SDK mode (no ROS master detected)"
fi
echo ""

echo "4. Reading intera.sh configuration..."
if [ -f ~/ros_ws/intera.sh ]; then
    echo "  Configuration:"
    grep "^robot_hostname=" ~/ros_ws/intera.sh | sed 's/^/    /'
    grep "^your_ip=" ~/ros_ws/intera.sh | sed 's/^/    /'
    grep "^ros_version=" ~/ros_ws/intera.sh | sed 's/^/    /'
else
    echo "    ✗ intera.sh not found"
fi
echo ""

echo "5. Testing intera.sh execution..."
echo "  Attempting to source intera.sh..."
cd ~/ros_ws
if ./intera.sh 2>&1 | head -20; then
    echo "    ℹ️  Check output above for success/failure"
else
    echo "    ✗ intera.sh execution failed"
fi
echo ""

echo "6. Checking if ROS environment was set..."
if [ -n "$ROS_MASTER_URI" ]; then
    echo "  ✓ ROS_MASTER_URI is set: $ROS_MASTER_URI"

    echo ""
    echo "7. Testing ROS connectivity..."
    echo "  Waiting 5 seconds for ROS to initialize..."
    sleep 5

    echo "  Attempting to list topics..."
    if timeout 10 rostopic list >/tmp/rostopic_list.txt 2>&1; then
        topic_count=$(wc -l < /tmp/rostopic_list.txt)
        echo "    ✓ Found $topic_count topics"
        echo "    Sample topics:"
        head -10 /tmp/rostopic_list.txt | sed 's/^/      /'

        echo ""
        echo "  Checking for key robot topics..."
        grep -q "/robot/joint_states" /tmp/rostopic_list.txt && echo "    ✓ /robot/joint_states found" || echo "    ✗ /robot/joint_states NOT found"
        grep -q "/robot/limb/right/endpoint_state" /tmp/rostopic_list.txt && echo "    ✓ /robot/limb/right/endpoint_state found" || echo "    ✗ /robot/limb/right/endpoint_state NOT found"
    else
        echo "    ✗ rostopic list failed or timed out"
        echo "    Error output:"
        cat /tmp/rostopic_list.txt | sed 's/^/      /'
    fi
else
    echo "  ✗ ROS_MASTER_URI is NOT set (intera.sh may have failed)"
    echo "  This means the robot connection was not established"
fi
echo ""

echo "=========================================="
echo "POST-ROBOT TESTS COMPLETE"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - If all tests passed, you can now control the robot"
echo "  - If intera.sh failed, check the debug output above"
echo "  - If ROS topics not found, robot may need to be in different mode"
