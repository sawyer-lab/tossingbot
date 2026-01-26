#!/bin/bash
# List all training and demo sessions
# Run from inside Docker container

# Find the script path (works from any directory inside container)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD="python3.8"
SCRIPT_PATH="$SCRIPT_DIR/src/tossingbot/scripts/session_tools.py"

echo "==================================="
echo "TossingBot Sessions"
echo "==================================="
echo ""

$PYTHON_CMD $SCRIPT_PATH list

echo ""
echo "To analyze a session, run:"
echo "  ./analyze_session.sh <session_id>"
echo ""
echo "To compare sessions, run:"
echo "  ./compare_sessions.sh <session_id1> <session_id2>"
echo ""
