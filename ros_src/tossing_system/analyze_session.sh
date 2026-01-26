#!/bin/bash
# Analyze a training session (generates plots and grasp visualizations)
# Run from inside Docker container

SESSION_ID="$1"

# Find the script path (works from any directory inside container)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD="python3.8"
SCRIPT_PATH="$SCRIPT_DIR/src/tossingbot/scripts/session_tools.py"
OUTPUT_BASE="$SCRIPT_DIR/sessions/training"

if [ -z "$SESSION_ID" ]; then
    echo "==================================="
    echo "TossingBot Session Analysis"
    echo "==================================="
    echo ""
    echo "Usage: $0 <session_id>"
    echo ""
    echo "Available sessions:"
    echo ""
    $PYTHON_CMD $SCRIPT_PATH list --mode training
    echo ""
    echo "Run: $0 <session_id>"
    exit 1
fi

echo "==================================="
echo "Analyzing Session: $SESSION_ID"
echo "==================================="
echo ""

# Run analysis
echo "Step 1/2: Generating training plots..."
$PYTHON_CMD $SCRIPT_PATH analyze --session "$SESSION_ID"

if [ $? -eq 0 ]; then
    echo "✓ Plots generated successfully"
else
    echo "✗ Analysis failed"
    exit 1
fi

echo ""
echo "Step 2/2: Generating grasp visualizations..."
$PYTHON_CMD $SCRIPT_PATH visualize --session "$SESSION_ID"

if [ $? -eq 0 ]; then
    echo "✓ Visualizations generated successfully"
else
    echo "✗ Visualization failed"
    exit 1
fi

echo ""
echo "==================================="
echo "Analysis Complete!"
echo "==================================="
echo ""
echo "Results saved to:"
echo "  $OUTPUT_BASE/$SESSION_ID/analysis/"
echo ""
echo "View plots:"
echo "  - success_rate.png"
echo "  - per_object_success.png"
echo "  - epsilon_decay.png"
echo "  - loss_curve.png"
echo "  - avg_confidence.png"
echo "  - actions_per_episode.png"
echo "  - grasp_heatmap.png"
echo ""
echo "View visualizations:"
echo "  - grasp_overlays/ (individual overlays)"
echo "  - success_montage.png"
echo "  - failure_montage.png"
echo ""
