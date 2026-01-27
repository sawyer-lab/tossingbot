#!/bin/bash
# Run complete train/test experiment from YAML config
# Usage: ./run_experiment.sh experiments/exp_baseline.yaml

CONFIG_FILE="$1"

# Find the script path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$CONFIG_FILE" ]; then
    echo "========================================="
    echo "TossingBot Experiment Runner"
    echo "========================================="
    echo ""
    echo "Usage: $0 <experiment_config.yaml>"
    echo ""
    echo "Available experiments:"
    ls "$SCRIPT_DIR/experiments/"*.yaml 2>/dev/null | xargs -n1 basename | sed 's/^/  /'
    echo ""
    echo "Example:"
    echo "  $0 experiments/exp_baseline.yaml"
    echo ""
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
fi

# Run inside container
python3.8 "$SCRIPT_DIR/src/tossingbot/scripts/run_experiment.py" "$CONFIG_FILE" --workspace "$SCRIPT_DIR"
