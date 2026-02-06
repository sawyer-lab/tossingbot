#!/usr/bin/env bash

NO_CACHE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--no-cache]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-cache]"
            exit 1
            ;;
    esac
done

# Get ethernet IP if available, otherwise fall back to first available IP
ETHERNET_INTERFACE="enp0s13f0u4c2"
if ip addr show "$ETHERNET_INTERFACE" &>/dev/null && ip addr show "$ETHERNET_INTERFACE" | grep -q "inet "; then
    HOST_IP=$(ip addr show "$ETHERNET_INTERFACE" | grep "inet " | awk '{print $2}' | cut -d'/' -f1)
    echo "Using ethernet IP: $HOST_IP"
else
    HOST_IP=$(hostname -I | cut -d ' ' -f 1)
    echo "Ethernet not available, using first IP: $HOST_IP"
fi

docker build \
    $NO_CACHE \
    --build-arg HOST_HOSTNAME=$(hostname) \
    --build-arg HOST_IP=$HOST_IP \
    --build-arg ROBOT_HOSTNAME=021607CP00070.local \
    -t robo2025-workspace workspace

