#!/bin/bash
# Attach to container and auto-configure intera.sh with runtime values

docker exec -it robo2025 bash -c '
    # Configure intera.sh with runtime environment variables (only if robot mode)
    if [ -f ~/init_robot.sh ] && [ -n "$HOST_IP" ] && [ -n "$ROBOT_IP" ]; then
        source ~/init_robot.sh 2>/dev/null || true
    fi
    # Start interactive bash
    exec bash
' "$@"
