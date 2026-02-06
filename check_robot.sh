#!/bin/bash
# Quick robot connection status check (no container restart)
# Run this from HOST

ROBOT_HOSTNAME="021607CP00070.local"
ETHERNET_INTERFACE="enp0s13f0u4c2"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

error() { echo -e "${RED}✗${NC} $1"; }
success() { echo -e "${GREEN}✓${NC} $1"; }
info() { echo -e "${YELLOW}ℹ${NC} $1"; }

echo "Quick Robot Status Check"
echo "========================"
echo ""

# Check ethernet
if ip addr show "$ETHERNET_INTERFACE" 2>/dev/null | grep -q "inet "; then
    ETH_IP=$(ip addr show "$ETHERNET_INTERFACE" | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
    success "Ethernet: $ETH_IP"
else
    error "Ethernet: DOWN or no IP"
fi

# Check robot
if command -v avahi-resolve &>/dev/null; then
    ROBOT_IP=$(avahi-resolve -4 -n "$ROBOT_HOSTNAME" 2>/dev/null | awk '{print $2}')
    if [ -n "$ROBOT_IP" ]; then
        if ping -c 1 -W 1 "$ROBOT_IP" &>/dev/null; then
            success "Robot: $ROBOT_IP (reachable)"
        else
            error "Robot: $ROBOT_IP (NOT reachable)"
        fi
    else
        error "Robot: Not detected"
    fi
else
    info "Robot: Cannot check (install avahi-utils)"
fi

# Check container
if docker ps -q -f name=robo2025 | grep -q .; then
    if docker exec robo2025 bash -c "ping -c 1 -W 1 ${ROBOT_IP:-192.168.1.103} &>/dev/null" 2>/dev/null; then
        success "Container: Running and can reach robot"
    else
        error "Container: Running but CANNOT reach robot"
    fi
else
    info "Container: Not running"
fi

echo ""
