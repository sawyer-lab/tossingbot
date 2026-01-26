#!/bin/bash
# Visualize per-object grasp distributions
# Run from inside Docker container

SESSION_ID="$1"

# Find the script path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD="python3.8"
SCRIPT_PATH="$SCRIPT_DIR/src/tossingbot/scripts/visualize_object_grasps.py"

if [ -z "$SESSION_ID" ]; then
    echo "==================================="
    echo "Per-Object Grasp Visualization"
    echo "==================================="
    echo ""
    echo "Usage: $0 <session_id>"
    echo ""
    echo "This creates object-relative grasp distribution plots."
    echo "Shows where the network grasps different object types."
    echo ""
    echo "Available sessions:"
    echo ""
    ls -1 sessions/training/ 2>/dev/null | sed 's/^/  /'
    echo ""
    exit 1
fi

SESSION_DIR="$SCRIPT_DIR/sessions/training/$SESSION_ID"
LOG_FILE="$SESSION_DIR/logs/training_log.jsonl"
OUTPUT_DIR="$SESSION_DIR/analysis/per_object_grasps"

if [ ! -f "$LOG_FILE" ]; then
    echo "Error: Log file not found: $LOG_FILE"
    exit 1
fi

echo "==================================="
echo "Analyzing: $SESSION_ID"
echo "==================================="
echo ""

$PYTHON_CMD $SCRIPT_PATH --log "$LOG_FILE" --output "$OUTPUT_DIR"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Visualization complete!"
    echo ""
    echo "Results saved to:"
    echo "  $OUTPUT_DIR/"
    echo ""
    echo "Files:"
    echo "  - all_objects_comparison.png  (side-by-side comparison)"
    echo "  - per_object/<object>_distribution.png  (individual plots)"
    echo ""
else
    echo "✗ Visualization failed"
    exit 1
fi
