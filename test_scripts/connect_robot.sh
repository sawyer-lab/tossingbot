#!/bin/bash
# Simple wrapper - check connection then run intera.sh
# Run this inside container when you want to connect to the robot

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Checking robot connection..."
echo ""

# Run pre-check
if "$SCRIPT_DIR/check_before_intera.sh"; then
    echo ""
    echo "All checks passed! Running intera.sh..."
    echo ""
    cd ~/ros_ws
    ./intera.sh
else
    echo ""
    echo "Pre-checks failed. Fix errors above before running intera.sh."
    exit 1
fi
