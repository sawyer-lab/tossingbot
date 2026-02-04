#!/bin/bash
# Run the release timing test
# Usage: ./run_timing_test.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "TossingBot: Release Timing Test"
echo "========================================="
echo "Note: Ensure Gazebo and Sawyer are running."
echo ""

# Run the test
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/test_release_timing.py"

if [ $? -eq 0 ]; then
    echo ""
    echo "Test completed successfully."
    echo "Running analysis..."
    "$SCRIPT_DIR/analyze_timing.sh"
else
    echo ""
    echo "Test failed or was interrupted."
    exit 1
fi
