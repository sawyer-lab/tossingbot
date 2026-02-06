#!/bin/bash
# Robust Robot Connection Script
# Automatically detects network, configures firewall, and starts container

set -e

# =============================================================================
# Configuration
# =============================================================================
ROBOT_IP="${1:-192.168.1.103}"
ROBOT_HOSTNAME="${2:-021607CP00070.local}"
CONTAINER_NAME="robo2025"
ROBOT_SUBNET="192.168.1"
IMAGE_NAME="robo2025-workspace:latest"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# =============================================================================
# Helper Functions
# =============================================================================
error() { echo -e "${RED}✗${NC} $1" >&2; }
success() { echo -e "${GREEN}✓${NC} $1"; }
info() { echo -e "${BLUE}ℹ${NC} $1"; }
warning() { echo -e "${YELLOW}⚠${NC} $1"; }

banner() {
    echo -e "${BLUE}=========================================="
    echo "ROBOT CONNECTION STARTUP"
    echo -e "==========================================${NC}"
}

# =============================================================================
# Dynamic Network Detection
# =============================================================================

detect_robot_interface() {
    info "Detecting robot network interface..." >&2

    # Find interface with robot subnet
    local interface=$(ip -br addr show | grep "${ROBOT_SUBNET}" | awk '{print $1}' | head -1)

    if [ -n "$interface" ]; then
        success "Found robot interface: $interface" >&2
        echo "$interface"
        return 0
    fi

    error "Could not find interface with subnet ${ROBOT_SUBNET}.x" >&2
    info "Available interfaces:" >&2
    ip -br addr show | grep -v "lo" | sed 's/^/  /' >&2
    exit 1
}

get_host_ip() {
    local interface=$1
    info "Getting host IP from $interface..." >&2

    local host_ip=$(ip addr show "$interface" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'/' -f1)

    if [ -n "$host_ip" ]; then
        success "Host IP: $host_ip" >&2
        echo "$host_ip"
        return 0
    fi

    error "Interface $interface has no IP address" >&2
    info "Waiting for DHCP..." >&2
    sleep 3

    # Try again after waiting
    host_ip=$(ip addr show "$interface" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
    if [ -n "$host_ip" ]; then
        success "Host IP: $host_ip" >&2
        echo "$host_ip"
        return 0
    fi

    error "Still no IP. Try: sudo dhclient $interface" >&2
    exit 1
}

verify_robot_reachable() {
    info "Verifying robot is reachable at $ROBOT_IP..." >&2

    if ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
        local latency=$(ping -c 3 "$ROBOT_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}' 2>/dev/null || echo "N/A")
        success "Robot is reachable (${latency}ms avg latency)" >&2
        return 0
    else
        warning "Robot not responding to ping (may have ICMP disabled)" >&2
        return 0
    fi
}

# =============================================================================
# Firewall Configuration
# =============================================================================

configure_firewall() {
    info "Configuring firewall for robot communication..." >&2

    # Check if rule already exists
    if sudo iptables -C INPUT -s "$ROBOT_IP" -j ACCEPT 2>/dev/null; then
        success "Firewall rule already exists for $ROBOT_IP" >&2
        return 0
    fi

    # Add rule
    info "Adding iptables rule to allow traffic from $ROBOT_IP..." >&2
    if sudo iptables -I INPUT -s "$ROBOT_IP" -j ACCEPT 2>&1; then
        success "Firewall configured (allows all traffic from robot)" >&2
    else
        error "Failed to configure firewall (needs sudo)" >&2
        exit 1
    fi
}

# =============================================================================
# Container Management
# =============================================================================

check_docker_image() {
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        error "Docker image '$IMAGE_NAME' not found" >&2
        info "Please run: ./build.sh" >&2
        exit 1
    fi
}

stop_existing_container() {
    if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
        info "Stopping existing container..." >&2
        docker stop "$CONTAINER_NAME" &>/dev/null
        success "Existing container stopped" >&2
    elif docker ps -aq -f name="$CONTAINER_NAME" | grep -q .; then
        info "Removing stopped container..." >&2
        docker rm "$CONTAINER_NAME" &>/dev/null
    fi
}

start_container() {
    local host_ip=$1

    info "Starting container with dynamic configuration..." >&2

    # Enable X11 forwarding
    xhost +local:root &>/dev/null || true

    # Start container - CRITICAL: --add-host for DNS resolution
    docker run -d --rm \
        --runtime=nvidia \
        --gpus all \
        --network host \
        --add-host "${ROBOT_HOSTNAME}:${ROBOT_IP}" \
        --device=/dev/bus/usb/001/002 \
        --privileged \
        -e DISPLAY="$DISPLAY" \
        -e TERM \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e __VK_LAYER_NV_optimus=NVIDIA_only \
        -e HOST_IP="$host_ip" \
        -e ROBOT_IP="$ROBOT_IP" \
        -e ROBOT_HOSTNAME="$ROBOT_HOSTNAME" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --mount type=bind,source=./ros_src/simulation,target=/simulation \
        --mount type=bind,source=./ros_src/custom_sawyer_description,target=/custom_sawyer_description \
        --mount type=bind,source=./ros_src/custom_sawyer_gazebo,target=/custom_sawyer_gazebo \
        --mount type=bind,source=./ros_src/environments,target=/environments \
        --mount type=bind,source=./ros_src/pneumatic_gripper_description,target=/pneumatic_gripper_description \
        --mount type=bind,source=./ros_src/depth_perception,target=/depth_perception \
        --mount type=bind,source=./ros_src/.vscode,target=/.vscode \
        --mount type=bind,source=./ros_src/plain_perception,target=/plain_perception \
        --mount type=bind,source=./ros_src/grasping,target=/grasping \
        --mount type=bind,source=./ros_src/tossing_system,target=/tossing_system \
        --mount type=bind,source=./test_scripts,target=/test_scripts \
        --name "$CONTAINER_NAME" \
        -it \
        "$IMAGE_NAME" > /dev/null 2>&1

    if [ $? -eq 0 ]; then
        success "Container started successfully" >&2
    else
        error "Failed to start container" >&2
        exit 1
    fi
}

verify_container_connectivity() {
    info "Verifying container can reach robot..." >&2
    sleep 2

    if docker exec "$CONTAINER_NAME" bash -c "ping -c 2 -W 2 $ROBOT_IP &>/dev/null"; then
        success "Container can reach robot" >&2
        return 0
    else
        error "Container cannot reach robot" >&2
        exit 1
    fi
}

# =============================================================================
# Main Execution
# =============================================================================

main() {
    banner
    echo ""

    # Help
    if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
        echo "Usage: $0 [ROBOT_IP] [ROBOT_HOSTNAME]"
        echo ""
        echo "Arguments:"
        echo "  ROBOT_IP        Robot IP address (default: 192.168.1.103)"
        echo "  ROBOT_HOSTNAME  Robot hostname (default: 021607CP00070.local)"
        echo ""
        echo "Example:"
        echo "  $0 192.168.1.105 sawyer-robot.local"
        exit 0
    fi

    # Pre-flight checks
    check_docker_image

    # Network detection
    ROBOT_INTERFACE=$(detect_robot_interface)
    HOST_IP=$(get_host_ip "$ROBOT_INTERFACE")

    echo ""
    info "Network Configuration:"
    echo "  Interface:      $ROBOT_INTERFACE"
    echo "  Host IP:        $HOST_IP"
    echo "  Robot IP:       $ROBOT_IP"
    echo "  Robot Hostname: $ROBOT_HOSTNAME"
    echo ""

    # Connectivity & Firewall
    verify_robot_reachable
    configure_firewall

    echo ""

    # Container management
    stop_existing_container
    start_container "$HOST_IP"
    verify_container_connectivity

    # Summary
    echo ""
    echo -e "${GREEN}=========================================="
    echo "✓ ROBOT STARTUP COMPLETE"
    echo -e "==========================================${NC}"
    echo ""
    echo "Configuration:"
    echo "  Host IP:        $HOST_IP"
    echo "  Robot IP:       $ROBOT_IP"
    echo "  Robot Hostname: $ROBOT_HOSTNAME"
    echo "  Interface:      $ROBOT_INTERFACE"
    echo "  Container:      $CONTAINER_NAME (running)"
    echo ""
    echo "Next steps:"
    echo "  1. Attach:  ./exec.sh"
    echo "  2. Connect: docker exec -it $CONTAINER_NAME bash -c 'source ~/init_robot.sh && cd ~/ros_ws && ./intera.sh'"
    echo ""
}

main "$@"
