#!/bin/bash
# Helper to find Sawyer robot on the local network
ROBOT_HOSTNAME="021607CP00070.local"

echo "--- Sawyer Discovery Tool ---"
echo "1. Checking mDNS (avahi)..."
if command -v avahi-resolve &>/dev/null; then
    IP=$(avahi-resolve -4 -n "$ROBOT_HOSTNAME" 2>/dev/null | awk '{print $2}')
    if [ -z "$IP" ]; then
        echo "   ✗ Could not resolve $ROBOT_HOSTNAME"
    else
        echo "   ✓ Found $ROBOT_HOSTNAME at $IP"
    fi
else
    echo "   ! avahi-resolve not installed"
fi

echo ""
echo "2. Scanning 192.168.1.0/24 for Rethink Robotics devices..."
if command -v nmap &>/dev/null; then
    # Try to find the vendor name in nmap scan
    sudo nmap -sn 192.168.1.0/24 | grep -B 2 "Rethink" || echo "   ? No Rethink devices found via nmap scan."
else
    echo "   ! nmap not installed. Manual ping test:"
    ping -c 1 -W 1 192.168.1.103
fi

echo ""
echo "Note: If the IP is different from 192.168.1.103, update it in:"
echo "      - Dockerfile"
echo "      - docker-compose.yml"
