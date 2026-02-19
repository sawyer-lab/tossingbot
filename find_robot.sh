#!/bin/bash
# Auto-discover robot IP and host IP, regardless of which network interface is used.
#
# Usage:
#   source <(./find_robot.sh)          # set ROBOT_IP / HOST_IP in current shell
#   eval "$(./find_robot.sh)"          # same thing
#   ./find_robot.sh                    # just print discovery results
#
# Outputs two lines to stdout (safe to eval):
#   ROBOT_IP=<ip>
#   HOST_IP=<ip>
# All diagnostic messages go to stderr.

ROBOT_HOSTNAME="${ROBOT_HOSTNAME:-021607CP00070.local}"
ROBOT_IP_HINT="${ROBOT_IP:-192.168.1.103}"   # last-known / fallback

# ── helpers ──────────────────────────────────────────────────────────────────

resolve_hostname() {
    local host="$1"

    # avahi-resolve (most reliable for .local mDNS) — force IPv4
    if command -v avahi-resolve &>/dev/null; then
        local ip
        ip=$(avahi-resolve -4 -n "$host" 2>/dev/null | awk '{print $2; exit}')
        [ -n "$ip" ] && { echo "$ip"; return 0; }
    fi

    # getent (uses NSS, works if avahi-daemon is running)
    local ip
    ip=$(getent hosts "$host" 2>/dev/null | awk '{print $1; exit}')
    [ -n "$ip" ] && { echo "$ip"; return 0; }

    # dig / nslookup as last resort
    if command -v dig &>/dev/null; then
        local ip
        ip=$(dig +short "$host" 2>/dev/null | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1)
        [ -n "$ip" ] && { echo "$ip"; return 0; }
    fi

    return 1
}

# Returns "<iface> <host_ip>" for the first interface that can reach target_ip.
find_reachable_interface() {
    local target_ip="$1"

    while IFS= read -r iface; do
        # Skip virtual / internal interfaces
        [[ "$iface" =~ ^(lo|docker|veth|br-|virbr|vmnet|tun|tap) ]] && continue

        # Must be UP
        ip link show "$iface" 2>/dev/null | grep -q "state UP" || continue

        # Must have an IPv4 address
        local host_ip
        host_ip=$(ip -4 addr show "$iface" 2>/dev/null \
                  | awk '/inet / {print $2}' | cut -d/ -f1 | head -1)
        [ -z "$host_ip" ] && continue

        echo "    Trying $iface ($host_ip) …" >&2
        if ping -c 1 -W 1 -I "$iface" "$target_ip" &>/dev/null; then
            echo "$iface $host_ip"
            return 0
        fi
    done < <(ip -o link show | awk -F': ' '{print $2}' | cut -d@ -f1)

    return 1
}

# ── main ─────────────────────────────────────────────────────────────────────

echo "Searching for Sawyer robot on the network …" >&2

# 1. Resolve robot hostname → IP
ROBOT_IP=""
resolved=$(resolve_hostname "$ROBOT_HOSTNAME" 2>/dev/null)
if [ -n "$resolved" ]; then
    ROBOT_IP="$resolved"
    echo "  mDNS resolved: $ROBOT_HOSTNAME → $ROBOT_IP" >&2
else
    echo "  mDNS resolution failed; using known IP: $ROBOT_IP_HINT" >&2
    ROBOT_IP="$ROBOT_IP_HINT"
fi

# 2. Find which host interface can reach the robot
echo "  Probing interfaces …" >&2
result=$(find_reachable_interface "$ROBOT_IP")

if [ -n "$result" ]; then
    IFACE=$(awk '{print $1}' <<< "$result")
    HOST_IP=$(awk '{print $2}' <<< "$result")
    echo "  ✓ Robot reachable via $IFACE  (host IP: $HOST_IP, robot IP: $ROBOT_IP)" >&2
else
    echo "  ✗ Cannot ping $ROBOT_IP from any interface." >&2
    echo "    Is the robot powered on and the ethernet cable connected?" >&2
    # Fall back to whatever is in .env / environment
    HOST_IP="${HOST_IP:-192.168.1.100}"
    echo "  Falling back to HOST_IP=$HOST_IP" >&2
fi

# 3. Emit KEY=VALUE pairs to stdout (eval-friendly)
echo "ROBOT_IP=$ROBOT_IP"
echo "HOST_IP=$HOST_IP"
