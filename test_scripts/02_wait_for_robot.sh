#!/bin/bash
# Monitor and detect when the Sawyer robot comes online
# Run this AFTER turning on the robot (while it boots)

ROBOT_HOSTNAME="021607CP00070.local"
ROBOT_IP="192.168.1.101"
MAX_WAIT=300  # 5 minutes

echo "=========================================="
echo "WAITING FOR ROBOT TO COME ONLINE"
echo "=========================================="
echo "Target: $ROBOT_HOSTNAME ($ROBOT_IP)"
echo "Max wait time: ${MAX_WAIT}s"
echo ""

start_time=$(date +%s)

# Function to check if robot is reachable
check_robot() {
    local method=$1
    case $method in
        "ping_ip")
            ping -c 1 -W 1 $ROBOT_IP >/dev/null 2>&1
            ;;
        "ping_hostname")
            ping -c 1 -W 1 $ROBOT_HOSTNAME >/dev/null 2>&1
            ;;
        "avahi")
            avahi-resolve -4 -n $ROBOT_HOSTNAME 2>/dev/null | grep -q "$ROBOT_IP"
            ;;
        "avahi_browse")
            timeout 2 avahi-browse -pt _rethink-robot._tcp 2>/dev/null | grep -q "IPv4.*021607CP00070"
            ;;
    esac
}

# Monitor loop
echo "Monitoring for robot..."
echo "(Press Ctrl+C to cancel)"
echo ""

method_status=("ping_ip:❌" "ping_hostname:❌" "avahi_resolve:❌" "avahi_browse:❌")

while true; do
    current_time=$(date +%s)
    elapsed=$((current_time - start_time))

    if [ $elapsed -gt $MAX_WAIT ]; then
        echo ""
        echo "⏱️  Timeout reached (${MAX_WAIT}s). Robot did not come online."
        exit 1
    fi

    # Check all methods
    all_passed=true

    if check_robot "ping_ip"; then
        method_status[0]="ping_ip:✓"
    else
        method_status[0]="ping_ip:❌"
        all_passed=false
    fi

    if check_robot "ping_hostname"; then
        method_status[1]="ping_hostname:✓"
    else
        method_status[1]="ping_hostname:❌"
        all_passed=false
    fi

    if check_robot "avahi"; then
        method_status[2]="avahi_resolve:✓"
    else
        method_status[2]="avahi_resolve:❌"
        all_passed=false
    fi

    if check_robot "avahi_browse"; then
        method_status[3]="avahi_browse:✓"
    else
        method_status[3]="avahi_browse:❌"
        all_passed=false
    fi

    # Print status
    printf "\r[%03ds] " $elapsed
    for status in "${method_status[@]}"; do
        printf "%s " "$status"
    done

    # If all methods pass, robot is online
    if [ "$all_passed" = true ]; then
        echo ""
        echo ""
        echo "🎉 ROBOT IS ONLINE! (detected after ${elapsed}s)"
        echo ""

        # Additional verification
        echo "Final verification:"
        echo "  Robot IP: $(avahi-resolve -4 -n $ROBOT_HOSTNAME 2>/dev/null || echo 'N/A')"
        echo "  Ping latency: $(ping -c 1 $ROBOT_IP 2>/dev/null | grep 'time=' | awk -F'time=' '{print $2}' || echo 'N/A')"
        echo ""

        exit 0
    fi

    sleep 1
done
