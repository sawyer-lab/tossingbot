#!/bin/bash
# Analyze an evaluation session
# Usage: ./analyze_eval.sh <training_session_name>

TRAIN_SESSION="$1"
EVAL_SESSION="${TRAIN_SESSION}_eval"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$TRAIN_SESSION" ]; then
    echo "========================================="
    echo "TossingBot Evaluation Analysis"
    echo "========================================="
    echo ""
    echo "Usage: $0 <training_session_name>"
    echo ""
    echo "This analyzes the evaluation session created from training."
    echo "Example: $0 exp_baseline (analyzes session_exp_baseline_eval)"
    echo ""
    echo "Available training sessions:"
    ls "$SCRIPT_DIR/sessions/training/" 2>/dev/null | grep -v "_eval$" | sed 's/^/  /'
    echo ""
    exit 1
fi

# Check if eval session exists
FULL_EVAL_SESSION="session_${EVAL_SESSION}"
if [ ! -d "$SCRIPT_DIR/sessions/training/$FULL_EVAL_SESSION" ]; then
    echo "Error: Evaluation session not found: $FULL_EVAL_SESSION"
    echo ""
    echo "Did you run evaluation yet?"
    echo "  ./evaluate.sh $TRAIN_SESSION <objects...>"
    exit 1
fi

echo "========================================================================"
echo "EVALUATION ANALYSIS: $TRAIN_SESSION"
echo "========================================================================"
echo "Training session: session_$TRAIN_SESSION"
echo "Eval session: $FULL_EVAL_SESSION"
echo "========================================================================"
echo ""

# Step 1: Generate generalization metrics
echo "Step 1/3: Computing generalization metrics..."
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/evaluate_generalization.py" \
    --session "$FULL_EVAL_SESSION"

if [ $? -ne 0 ]; then
    echo "✗ Metrics computation failed"
    exit 1
fi
echo "✓ Metrics computed"
echo ""

# Step 2: Analyze eval session
echo "Step 2/3: Generating evaluation plots..."
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/session_tools.py" \
    analyze --session "$FULL_EVAL_SESSION"

if [ $? -ne 0 ]; then
    echo "✗ Analysis failed"
    exit 1
fi
echo "✓ Plots generated"
echo ""

# Step 3: Generate per-object visualizations
echo "Step 3/3: Generating per-object grasp visualizations..."
"$SCRIPT_DIR/visualize_object_grasps.sh" "$FULL_EVAL_SESSION"

if [ $? -ne 0 ]; then
    echo "✗ Visualization failed"
    exit 1
fi

echo ""
echo "========================================================================"
echo "ANALYSIS COMPLETE!"
echo "========================================================================"
echo ""
echo "Results saved to:"
echo "  $SCRIPT_DIR/sessions/training/$FULL_EVAL_SESSION/analysis/"
echo ""
echo "Key files:"
echo "  - Generalization metrics in session metadata"
echo "  - Success rate plots"
echo "  - Per-object grasp distributions"
echo ""
