#!/bin/bash
# Analyze timing logs and visualize trajectories
# Usage: ./analyze_timing.sh [log_file.json]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs/timing_test"

# If argument provided, use it. Otherwise, use latest.
if [ -n "$1" ]; then
    LOG_FILE="$1"
    # If it's just a filename, prepend LOG_DIR
    if [[ "$LOG_FILE" != /* ]]; then
        LOG_FILE="$LOG_DIR/$LOG_FILE"
    fi
else
    # Find latest json in logs/timing_test
    LOG_FILE=$(ls -t "$LOG_DIR"/*.json 2>/dev/null | head -n 1)
fi

if [ -z "$LOG_FILE" ] || [ ! -f "$LOG_FILE" ]; then
    echo "Error: No log file found in $LOG_DIR"
    echo "Run the timing test first: ./run_timing_test.sh"
    exit 1
fi

echo "========================================="
echo "Analyzing Timing Log: $(basename "$LOG_FILE")"
echo "========================================="

# Run the standard timing analysis (Velocity Loss vs Offset)
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/analyze_timing.py" "$LOG_FILE"

# Run the trajectory visualization
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/visualize_trajectory.py" "$LOG_FILE"

# Run the animation generator
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/animate_trajectory.py" "$LOG_FILE"

echo ""
echo "========================================="
echo "Analysis Complete!"
echo "Plots and animations saved in same directory as log."
echo "========================================="
