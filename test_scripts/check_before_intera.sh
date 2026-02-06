#!/bin/bash
# Run INSIDE container BEFORE sourcing intera.sh
# Checks all prerequisites for successful robot connection

ROBOT_HOSTNAME="021607CP00070.local"
EXPECTED_HOST_IP="192.168.1.100"
INTERA_PATH=~/ros_ws/intera.sh

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

error() { echo -e "${RED}✗${NC} $1"; }
success() { echo -e "${GREEN}✓${NC} $1"; }
warning() { echo -e "${YELLOW}⚠${NC} $1"; }
info() { echo -e "${YELLOW}ℹ${NC} $1"; }

ERRORS=0
WARNINGS=0

echo "=========================================="
echo "PRE-INTERA.SH CONNECTION CHECK"
echo "=========================================="
echo ""

# Check 1: Network interfaces
echo "1. Checking network interfaces..."
if ip addr show enp0s13f0u4c2 &>/dev/null; then
    CONTAINER_IP=$(ip addr show enp0s13f0u4c2 | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
    if [ -n "$CONTAINER_IP" ]; then
        success "Ethernet interface UP with IP: $CONTAINER_IP"
        if [ "$CONTAINER_IP" == "$EXPECTED_HOST_IP" ]; then
            success "IP matches expected: $EXPECTED_HOST_IP"
        else
            warning "IP is $CONTAINER_IP (expected $EXPECTED_HOST_IP)"
            ((WARNINGS++))
        fi
    else
        error "Ethernet interface UP but no IP"
        ((ERRORS++))
    fi
else
    error "Cannot see ethernet interface enp0s13f0u4c2"
    info "This is expected if not using --network host"
    ((ERRORS++))
fi
echo ""

# Check 2: Gateway reachability
echo "2. Testing gateway connectivity..."
if ping -c 1 -W 1 192.168.1.1 &>/dev/null; then
    success "Can reach gateway (192.168.1.1)"
else
    error "Cannot reach gateway - check network"
    ((ERRORS++))
fi
echo ""

# Check 3: Robot detection via avahi
echo "3. Detecting robot via avahi..."
ROBOT_IP=""
if command -v avahi-resolve &>/dev/null; then
    if timeout 3 avahi-browse -pt _rethink-robot._tcp 2>/dev/null | grep -q "IPv4.*021607CP00070"; then
        success "Robot detected via avahi"
        ROBOT_IP=$(avahi-resolve -4 -n "$ROBOT_HOSTNAME" 2>/dev/null | awk '{print $2}')
        if [ -n "$ROBOT_IP" ]; then
            success "Robot hostname resolves to: $ROBOT_IP"
        else
            warning "Robot detected but cannot resolve IP"
            ((WARNINGS++))
        fi
    else
        error "Robot NOT detected via avahi"
        info "Make sure robot is powered on and fully booted"
        ((ERRORS++))
    fi
else
    warning "avahi-resolve not available"
    info "Will try ping with known IP"
    ROBOT_IP="192.168.1.103"  # Last known IP
    ((WARNINGS++))
fi
echo ""

# Check 4: Robot ping test
echo "4. Testing robot connectivity..."
if [ -n "$ROBOT_IP" ]; then
    if ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
        LATENCY=$(ping -c 3 "$ROBOT_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}')
        success "Robot reachable at $ROBOT_IP (avg ${LATENCY}ms)"

        # Try hostname too
        if ping -c 1 -W 1 "$ROBOT_HOSTNAME" &>/dev/null; then
            success "Robot reachable via hostname: $ROBOT_HOSTNAME"
        else
            warning "Robot IP works but hostname doesn't resolve"
            info "You may need to use IP in intera.sh instead of hostname"
            ((WARNINGS++))
        fi
    else
        error "Cannot ping robot at $ROBOT_IP"
        info "Check: Is robot fully booted? Is ethernet cable connected?"
        ((ERRORS++))
    fi
else
    error "Could not determine robot IP"
    ((ERRORS++))
fi
echo ""

# Check 5: intera.sh configuration
echo "5. Checking intera.sh configuration..."
if [ -f "$INTERA_PATH" ]; then
    success "intera.sh found at $INTERA_PATH"

    # Check your_ip
    YOUR_IP=$(grep "^your_ip=" "$INTERA_PATH" | head -1 | cut -d'"' -f2)
    if [ -n "$YOUR_IP" ] && [ "$YOUR_IP" != "" ]; then
        success "your_ip is set: $YOUR_IP"
        if [ "$YOUR_IP" == "$EXPECTED_HOST_IP" ]; then
            success "your_ip matches expected value"
        else
            warning "your_ip is $YOUR_IP (expected $EXPECTED_HOST_IP)"
            ((WARNINGS++))
        fi
    else
        error "your_ip is EMPTY - intera.sh will fail!"
        info "Edit ~/ros_ws/intera.sh and set: your_ip=\"$EXPECTED_HOST_IP\""
        ((ERRORS++))
    fi

    # Check robot_hostname
    ROBOT_HOST_CONFIG=$(grep "^robot_hostname=" "$INTERA_PATH" | head -1 | cut -d'"' -f2)
    if [ -n "$ROBOT_HOST_CONFIG" ]; then
        success "robot_hostname is set: $ROBOT_HOST_CONFIG"

        # Check if it's still the placeholder (robot_hostname.local is the actual placeholder)
        if [ "$ROBOT_HOST_CONFIG" == "robot_hostname.local" ]; then
            error "robot_hostname is still the default placeholder!"
            info "Update to actual hostname: 021607CP00070.local or IP: $ROBOT_IP"
            ((ERRORS++))
        fi
    else
        error "robot_hostname is not set"
        ((ERRORS++))
    fi

    # Check ros_version
    ROS_VER=$(grep "^ros_version=" "$INTERA_PATH" | head -1 | cut -d'"' -f2)
    if [ "$ROS_VER" == "melodic" ]; then
        success "ros_version is set to melodic"
    else
        warning "ros_version is: $ROS_VER (expected melodic)"
        ((WARNINGS++))
    fi
else
    error "intera.sh not found at $INTERA_PATH"
    ((ERRORS++))
fi
echo ""

# Check 6: ROS installation
echo "6. Checking ROS installation..."
if [ -d "/opt/ros/melodic" ]; then
    success "ROS melodic installation found"
    if [ -f "/opt/ros/melodic/setup.bash" ]; then
        success "ROS setup.bash exists"
    else
        error "ROS setup.bash not found"
        ((ERRORS++))
    fi
else
    error "ROS melodic not found in /opt/ros"
    ((ERRORS++))
fi
echo ""

# Check 7: Catkin workspace
echo "7. Checking catkin workspace..."
if [ -f ~/ros_ws/devel/setup.bash ]; then
    success "Catkin workspace built (devel/setup.bash exists)"
else
    error "Catkin workspace not built"
    info "Run: cd ~/ros_ws && catkin_make"
    ((ERRORS++))
fi
echo ""

# Summary
echo "=========================================="
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✓ ALL CHECKS PASSED${NC}"
    echo "=========================================="
    echo ""
    echo "You can now run:"
    echo "  cd ~/ros_ws && ./intera.sh"
    echo ""
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠ PASSED WITH $WARNINGS WARNING(S)${NC}"
    echo "=========================================="
    echo ""
    echo "You can proceed, but there may be issues."
    echo "Review warnings above."
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Continuing..."
        echo "Run: cd ~/ros_ws && ./intera.sh"
        exit 0
    else
        exit 1
    fi
else
    echo -e "${RED}✗ FAILED WITH $ERRORS ERROR(S) and $WARNINGS WARNING(S)${NC}"
    echo "=========================================="
    echo ""
    echo "Cannot proceed - fix errors above first."
    echo ""
    echo "Common fixes:"
    echo "  - Ensure ethernet cable is connected"
    echo "  - Ensure robot is powered on and fully booted"
    echo "  - Rebuild container: exit, then ./build.sh on host"
    echo "  - Manually edit intera.sh: nano ~/ros_ws/intera.sh"
    echo ""
    exit 1
fi
