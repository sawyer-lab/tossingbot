#!/bin/bash
# Evaluate trained model on test objects
# Usage: ./evaluate.sh <session_name> <object1> <object2> ... [--episodes N]

SESSION="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$SESSION" ]; then
    echo "========================================="
    echo "TossingBot Evaluation"
    echo "========================================="
    echo ""
    echo "Usage: $0 <session_name> <object1> <object2> ... [--episodes N]"
    echo ""
    echo "Examples:"
    echo "  $0 exp_baseline L_shape I_shape T_shape"
    echo "  $0 exp_baseline puck bolt C_shape --episodes 50"
    echo ""
    echo "Available sessions:"
    ls "$SCRIPT_DIR/sessions/training/" 2>/dev/null | grep -v "^$" | sed 's/^/  /'
    echo ""
    exit 1
fi

shift  # Remove session name from args

# Parse objects and episodes
OBJECTS=()
EPISODES=100

while [[ $# -gt 0 ]]; do
    case $1 in
        --episodes)
            EPISODES="$2"
            shift 2
            ;;
        *)
            OBJECTS+=("$1")
            shift
            ;;
    esac
done

if [ ${#OBJECTS[@]} -eq 0 ]; then
    echo "Error: No objects specified"
    echo "Usage: $0 <session_name> <object1> <object2> ..."
    exit 1
fi

echo "========================================================================"
echo "EVALUATION: $SESSION"
echo "========================================================================"
echo "Objects: ${OBJECTS[*]}"
echo "Episodes: $EPISODES"
echo "========================================================================"
echo ""

python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/auto_grasp.py" \
    --eval-only \
    --session "$SESSION" \
    --eval-objects "${OBJECTS[@]}" \
    --eval-episodes "$EPISODES"
