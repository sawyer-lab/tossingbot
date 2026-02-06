#!/bin/bash
# Host-side script to check robot connectivity and start container
# Run this from the HOST (not inside container)

set -e

ROBOT_HOSTNAME="021607CP00070.local"
ETHERNET_INTERFACE="enp0s13f0u4c2"
EXPECTED_HOST_SUBNET="192.168.1"
CONTAINER_NAME="robo2025"

echo "=========================================="
echo "ROBOT CONNECTION & CONTAINER STARTUP"
echo "=========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

error() { echo -e "${RED}✗${NC} $1"; }
success() { echo -e "${GREEN}✓${NC} $1"; }
info() { echo -e "${YELLOW}ℹ${NC} $1"; }

# Step 1: Check ethernet interface
echo "1. Checking ethernet connection..."
if ip addr show "$ETHERNET_INTERFACE" &>/dev/null; then
    ETHERNET_IP=$(ip addr show "$ETHERNET_INTERFACE" | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
    if [ -n "$ETHERNET_IP" ]; then
        success "Ethernet interface $ETHERNET_INTERFACE is UP with IP: $ETHERNET_IP"

        # Check if it's on the right subnet
        if [[ "$ETHERNET_IP" == $EXPECTED_HOST_SUBNET* ]]; then
            success "IP is on correct subnet ($EXPECTED_HOST_SUBNET.x)"
        else
            error "IP is NOT on expected subnet (expected $EXPECTED_HOST_SUBNET.x, got $ETHERNET_IP)"
            echo "  Check your network configuration"
            exit 1
        fi
    else
        error "Ethernet interface $ETHERNET_INTERFACE is UP but has no IP address"
        echo "  Try: sudo dhclient $ETHERNET_INTERFACE"
        exit 1
    fi
else
    error "Ethernet interface $ETHERNET_INTERFACE not found or DOWN"
    echo "  Available interfaces:"
    ip -br addr show | grep -v "lo" | sed 's/^/    /'
    exit 1
fi
echo ""

# Step 2: Check for robot via avahi
echo "2. Detecting robot..."
if command -v avahi-browse &>/dev/null; then
    if timeout 3 avahi-browse -pt _rethink-robot._tcp 2>/dev/null | grep -q "IPv4.*021607CP00070"; then
        success "Robot detected via avahi (_rethink-robot._tcp)"

        # Get robot IP
        ROBOT_IP=$(avahi-resolve -4 -n "$ROBOT_HOSTNAME" 2>/dev/null | awk '{print $2}')
        if [ -n "$ROBOT_IP" ]; then
            success "Robot IP resolved: $ROBOT_IP"
        else
            error "Could not resolve robot IP from hostname $ROBOT_HOSTNAME"
            exit 1
        fi
    else
        error "Robot not detected via avahi"
        info "Make sure robot is powered on and fully booted"
        echo ""
        echo "Scanning network for any devices..."
        nmap -sn ${EXPECTED_HOST_SUBNET}.0/24 2>/dev/null | grep "Nmap scan report" | sed 's/^/  /'
        exit 1
    fi
else
    error "avahi-browse not installed, trying nmap..."
    # Fallback: assume .103 or scan
    ROBOT_IP="${EXPECTED_HOST_SUBNET}.103"
    info "Assuming robot IP: $ROBOT_IP (install avahi-utils for auto-detection)"
fi
echo ""

# Step 3: Test connectivity to robot
echo "3. Testing connectivity to robot..."
if ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
    LATENCY=$(ping -c 3 "$ROBOT_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}')
    success "Robot is reachable at $ROBOT_IP (avg latency: ${LATENCY}ms)"
else
    error "Cannot ping robot at $ROBOT_IP"
    echo "  Troubleshooting:"
    echo "    - Is robot powered on?"
    echo "    - Is ethernet cable connected?"
    echo "    - Is robot fully booted? (check screen)"
    exit 1
fi
echo ""

# Step 4: Check/stop existing container
echo "4. Checking existing container..."
if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
    info "Container '$CONTAINER_NAME' is already running"
    read -p "  Restart container? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "  Stopping container..."
        docker stop "$CONTAINER_NAME" &>/dev/null
        success "Container stopped"
    else
        info "Keeping existing container running"
        echo ""
        echo "To attach to container: ./exec.sh"
        exit 0
    fi
elif docker ps -aq -f name="$CONTAINER_NAME" | grep -q .; then
    info "Container '$CONTAINER_NAME' exists but is stopped, removing..."
    docker rm "$CONTAINER_NAME" &>/dev/null
fi
echo ""

# Step 5: Start container
echo "5. Starting container..."
if ./run.sh &>/dev/null; then
    success "Container '$CONTAINER_NAME' started successfully"
else
    error "Failed to start container"
    echo "  Check ./run.sh for errors"
    exit 1
fi
echo ""

# Step 6: Verify container can reach robot
echo "6. Verifying container connectivity..."
sleep 2  # Wait for container to initialize
if docker exec "$CONTAINER_NAME" bash -c "ping -c 2 -W 2 $ROBOT_IP &>/dev/null"; then
    success "Container can reach robot at $ROBOT_IP"
else
    error "Container CANNOT reach robot"
    info "Checking container routing..."
    docker exec "$CONTAINER_NAME" ip route show | sed 's/^/    /'
    exit 1
fi
echo ""

# Step 7: Summary
echo "=========================================="
echo "✓ ALL CHECKS PASSED"
echo "=========================================="
echo ""
echo "Summary:"
echo "  Host IP:      $ETHERNET_IP"
echo "  Robot IP:     $ROBOT_IP"
echo "  Robot Host:   $ROBOT_HOSTNAME"
echo "  Container:    $CONTAINER_NAME (running)"
echo ""
echo "Next steps:"
echo "  1. Attach to container:  ./exec.sh"
echo "  2. Edit intera.sh:       nano ~/ros_ws/intera.sh"
echo "     - Set: your_ip=\"$ETHERNET_IP\""
echo "     - Set: robot_hostname=\"$ROBOT_HOSTNAME\""
echo "  3. Source intera.sh:     cd ~/ros_ws && ./intera.sh"
echo ""
echo "Or run automated tests:    cd /test_scripts && ./run_all_tests.sh"
echo ""
