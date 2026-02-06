#!/bin/bash
# Ensures avahi-daemon is running for mDNS resolution
# May need to be run as root or with sudo

echo "=========================================="
echo "AVAHI DAEMON SETUP"
echo "=========================================="
echo ""

echo "Checking avahi-daemon status..."
if pgrep -x "avahi-daemon" > /dev/null; then
    echo "  ✓ avahi-daemon is already running"
    ps aux | grep avahi-daemon | grep -v grep
else
    echo "  ✗ avahi-daemon is not running"
    echo ""
    echo "  Attempting to start..."

    # Try to start with service command
    if command -v service >/dev/null 2>&1; then
        echo "  Using 'service' command..."
        service avahi-daemon start 2>&1
    fi

    # Check if it started
    sleep 1
    if pgrep -x "avahi-daemon" > /dev/null; then
        echo "  ✓ avahi-daemon started successfully"
    else
        echo "  ✗ Failed to start avahi-daemon"
        echo ""
        echo "  This is expected in some Docker configurations."
        echo "  mDNS (.local hostnames) may not work without it."
        echo "  You can still use IP address: 192.168.1.101"
    fi
fi

echo ""
echo "Testing mDNS resolution..."
if avahi-resolve -4 -n 021607CP00070.local 2>/dev/null | grep -q "192.168.1.101"; then
    echo "  ✓ mDNS resolution working"
else
    echo "  ✗ mDNS resolution not working yet"
    echo "  (Robot may not be powered on)"
fi

echo ""
echo "=========================================="
