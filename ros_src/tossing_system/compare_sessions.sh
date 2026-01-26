#!/bin/bash
# Compare multiple training sessions side-by-side
# Run from inside Docker container

# Find the script path (works from any directory inside container)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD="python3.8"
SCRIPT_PATH="$SCRIPT_DIR/src/tossingbot/scripts/session_tools.py"

if [ $# -lt 2 ]; then
    echo "==================================="
    echo "TossingBot Session Comparison"
    echo "==================================="
    echo ""
    echo "Usage: $0 <session_id1> <session_id2> [session_id3 ...]"
    echo ""
    echo "Available sessions:"
    echo ""
    $PYTHON_CMD $SCRIPT_PATH list --mode training
    echo ""
    echo "Example: $0 session_baseline session_experiment_v2"
    exit 1
fi

echo "==================================="
echo "Comparing Sessions"
echo "==================================="
echo ""

# Run comparison
$PYTHON_CMD $SCRIPT_PATH compare --sessions "$@"

echo ""
