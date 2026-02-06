# TossingBot Multi-Eval Experiment System

This document describes the new unified experiment system that supports **one training phase with multiple evaluation phases**.

## Overview

The new experiment system allows you to:
- **Define complete experiments** in a single YAML config file
- **Train once** and **evaluate many times** on different object sets
- **Compare performance** across multiple evaluation scenarios
- **Run in containers** with proper path resolution
- **Generate unified analysis** combining all phases

### Key Innovation: Train Once, Evaluate Many

```
Experiment = 1 Training + N Evaluations

exp_baseline/
├── train/              ← One training phase
│   ├── checkpoints/
│   ├── buffer/
│   └── logs/
├── evals/              ← Multiple evaluation phases
│   ├── seen/
│   ├── unseen_simple/
│   ├── unseen_complex/
│   └── generalization/
└── analysis/           ← Unified analysis of all phases
```

## Quick Start

### 1. Create Experiment Config

Create a YAML file in `experiments/` directory:

```yaml
# experiments/my_experiment.yaml
name: my_experiment
description: "Test generalization on new objects"

train:
  objects: [L_shape, I_shape, T_shape]
  steps: 1500

evals:
  seen:
    description: "Test on training objects"
    objects: [L_shape, I_shape, T_shape]
    episodes: 100
    checkpoint: best

  unseen_simple:
    description: "Test on simple shapes"
    objects: [cube, cylinder, sphere]
    episodes: 100
    checkpoint: best

  unseen_complex:
    description: "Test on complex shapes"
    objects: [C_shape, cross, puck]
    episodes: 100
    checkpoint: best
```

### 2. Run Experiment

```bash
# Option A: Run everything (train + all evals + analyze)
python src/tossingbot/scripts/experiment_cli.py run \
  --experiment my_experiment \
  --phase all

# Option B: Run phases separately
# Step 1: Train
python src/tossingbot/scripts/experiment_cli.py run \
  --experiment my_experiment \
  --phase train

# Step 2: Run all evaluations
python src/tossingbot/scripts/experiment_cli.py run \
  --experiment my_experiment \
  --phase eval \
  --eval-name all

# Step 3: Analyze results
python src/tossingbot/scripts/experiment_cli.py analyze \
  --experiment my_experiment
```

### 3. View Results

```bash
# Show experiment info
python src/tossingbot/scripts/experiment_cli.py info \
  --experiment my_experiment

# List all evaluation phases
python src/tossingbot/scripts/experiment_cli.py list-evals \
  --experiment my_experiment

# Results are saved to:
sessions/experiments/my_experiment/
├── train/logs/training_log.jsonl
├── evals/
│   ├── seen/logs/evaluation_log.jsonl
│   ├── unseen_simple/logs/evaluation_log.jsonl
│   └── unseen_complex/logs/evaluation_log.jsonl
└── analysis/
    ├── train_plots/
    ├── eval_seen_plots/
    ├── eval_unseen_simple_plots/
    ├── eval_unseen_complex_plots/
    ├── comparison_plots/          ← Compare across evals
    └── metrics.json               ← All metrics
```

## CLI Commands

### Create Experiment

```bash
# Create from YAML config
python experiment_cli.py create --config experiments/my_exp.yaml
```

### List Experiments

```bash
# List all experiments
python experiment_cli.py list

# Filter by status
python experiment_cli.py list --status completed
```

### Show Info

```bash
# Show experiment details
python experiment_cli.py info --experiment my_exp

# Show detailed info
python experiment_cli.py info --experiment my_exp --verbose

# List evaluation phases
python experiment_cli.py list-evals --experiment my_exp
```

### Run Phases

```bash
# Run training phase
python experiment_cli.py run --experiment my_exp --phase train

# Run specific eval phase
python experiment_cli.py run --experiment my_exp --phase eval --eval-name seen

# Run all eval phases
python experiment_cli.py run --experiment my_exp --phase eval --eval-name all

# Run everything (train + all evals)
python experiment_cli.py run --experiment my_exp --phase all
```

### Analyze Results

```bash
# Analyze all phases
python experiment_cli.py analyze --experiment my_exp

# Analyze specific eval phases
python experiment_cli.py analyze --experiment my_exp --eval-names seen unseen_simple
```

### Compare Experiments

```bash
# Compare overall metrics
python experiment_cli.py compare --experiments exp1 exp2 exp3

# Compare specific eval phase across experiments
python experiment_cli.py compare --experiments exp1 exp2 --eval-name seen
```

### Archive/Delete

```bash
# Archive experiment
python experiment_cli.py archive --experiment old_exp

# Delete experiment (with confirmation)
python experiment_cli.py delete --experiment bad_exp

# Force delete (no confirmation)
python experiment_cli.py delete --experiment bad_exp --force
```

## Configuration File Format

### Full Example

```yaml
# Experiment metadata
name: exp_generalization_v1
description: "Comprehensive generalization test"

# Training phase (required)
train:
  objects: [L_shape, I_shape, T_shape]  # Training objects
  steps: 1500                            # Target training steps
  hyperparameters:                       # Optional
    learning_rate: 0.0001
    batch_size: 32
    gamma: 0.99

# Evaluation phases (required, at least one)
evals:
  # Eval phase name (alphanumeric + underscore)
  seen:
    description: "Test on training objects"
    objects: [L_shape, I_shape, T_shape]  # Objects for this eval
    episodes: 100                          # Number of episodes
    checkpoint: best                       # Which checkpoint to use

  unseen_simple:
    description: "Test on simple shapes"
    objects: [cube, cylinder, sphere]
    episodes: 100
    checkpoint: best

  # You can add as many eval phases as needed
  unseen_complex:
    description: "Test on complex shapes"
    objects: [C_shape, cross, puck]
    episodes: 100
    checkpoint: best

# Container configuration (optional)
container:
  image: tossingbot:latest
  mount_paths:
    sessions: /workspace/sessions
    logs: /workspace/logs
  ros_master_uri: http://localhost:11311

# Analysis configuration (optional)
analysis:
  moving_average_window: 50
  report_format: pdf
```

### Checkpoint Options

The `checkpoint` field in eval phases can be:
- `best` - Use the best performing checkpoint from training (default)
- `latest` - Use the most recent checkpoint
- `step_N` - Use checkpoint at specific step (e.g., `step_500`, `step_1000`)

### Required Fields

**Training section (required):**
- `objects` - List of object names (at least 1)
- `steps` - Target number of training steps (optional, can run indefinitely)

**Each eval phase (at least 1 required):**
- `objects` - List of object names (at least 1)
- `episodes` - Number of evaluation episodes

## Example Use Cases

### 1. Basic Generalization Test

Train on complex shapes, test on seen vs unseen:

```yaml
name: exp_basic_gen
train:
  objects: [L_shape, I_shape, T_shape]
  steps: 1500
evals:
  seen:
    objects: [L_shape, I_shape, T_shape]
    episodes: 100
  unseen:
    objects: [cube, cylinder, sphere]
    episodes: 100
```

### 2. Progressive Complexity

Test generalization from simple to complex:

```yaml
name: exp_progressive
train:
  objects: [cube, cylinder, bar]  # Simple shapes
  steps: 2000
evals:
  seen_simple:
    objects: [cube, cylinder, bar]
    episodes: 100
  unseen_simple:
    objects: [sphere, puck]
    episodes: 100
  moderate:
    objects: [L_shape, T_shape]
    episodes: 100
  complex:
    objects: [C_shape, cross]
    episodes: 100
```

### 3. Checkpoint Comparison

Compare performance at different training stages:

```yaml
name: exp_checkpoint_comp
train:
  objects: [L_shape, I_shape, T_shape]
  steps: 2000
evals:
  early:
    objects: [cube, sphere]
    episodes: 100
    checkpoint: step_500
  mid:
    objects: [cube, sphere]
    episodes: 100
    checkpoint: step_1000
  final:
    objects: [cube, sphere]
    episodes: 100
    checkpoint: best
```

## Container Execution

The system automatically detects if running in a container and adjusts paths.

### Docker Compose Setup

```yaml
# docker-compose.yaml
services:
  tossingbot:
    image: tossingbot:latest
    volumes:
      - ./sessions:/workspace/sessions          # Persist experiments
      - ./experiments:/workspace/experiments    # Mount configs
    environment:
      - ROS_MASTER_URI=http://rosmaster:11311
      - CONTAINER=true
```

### Running in Container

```bash
# Run experiment in container
docker run --rm -it \
  -v $(pwd)/sessions:/workspace/sessions \
  -v $(pwd)/experiments:/workspace/experiments \
  --network ros-network \
  tossingbot:latest \
  python experiment_cli.py run --experiment my_exp --phase all

# Results persist in ./sessions/experiments/my_exp/
```

## Directory Structure

```
tossing_system/
├── experiments/                        # Experiment configs
│   ├── exp_baseline.yaml
│   ├── exp_generalization.yaml
│   └── exp_progressive.yaml
│
├── sessions/
│   └── experiments/                    # Experiment data
│       └── <experiment_id>/
│           ├── config.yaml             # Experiment config
│           ├── metadata.json           # Runtime state
│           ├── train/                  # Training artifacts
│           │   ├── checkpoints/
│           │   │   ├── checkpoint_best.pth
│           │   │   ├── checkpoint_latest.pth
│           │   │   └── checkpoint_step_*.pth
│           │   ├── buffer/
│           │   │   └── replay_buffer.pkl
│           │   └── logs/
│           │       └── training_log.jsonl
│           ├── evals/                  # Evaluation artifacts
│           │   ├── <eval_name_1>/
│           │   │   └── logs/
│           │   │       └── evaluation_log.jsonl
│           │   ├── <eval_name_2>/
│           │   └── ...
│           └── analysis/               # Analysis outputs
│               ├── train_plots/
│               ├── eval_<name>_plots/
│               ├── comparison_plots/
│               ├── metrics.json
│               └── summary.txt
│
└── src/tossingbot/
    ├── learning/
    │   ├── experiment_session.py       # NEW: ExperimentSession class
    │   ├── experiment_manager.py       # NEW: ExperimentManager class
    │   └── session_manager.py          # OLD: Kept for compatibility
    ├── scripts/
    │   ├── experiment_cli.py           # NEW: Unified CLI
    │   ├── analyze_experiment.py       # NEW: Multi-eval analysis
    │   └── auto_grasp.py               # Modified to support experiments
    └── utils/
        └── container_paths.py          # NEW: Container detection
```

## Backward Compatibility

The old session-based system is still supported:

```bash
# Old system (still works)
python auto_grasp.py --mode training --session old_session

# New system
python experiment_cli.py run --experiment new_exp --phase train
```

## Analysis Outputs

After running analysis, you'll find:

### 1. Training Plots
- `train_plots/success_rate.png` - Success rate over time
- `train_plots/loss_curve.png` - Training loss
- `train_plots/per_object_success.png` - Per-object performance
- `train_plots/confidence_distribution.png` - Network confidence
- And more...

### 2. Per-Eval Plots
For each eval phase (e.g., `seen`, `unseen_simple`):
- `eval_<name>_plots/per_object_success.png`
- `eval_<name>_plots/confidence_distribution.png`
- `eval_<name>_plots/grasp_heatmap.png`
- And more...

### 3. Comparison Plots
When multiple eval phases exist:
- `comparison_plots/eval_comparison_success.png` - Bar chart comparing success rates
- `comparison_plots/eval_comparison_attempts.png` - Attempt counts
- `comparison_plots/eval_comparison_per_object.png` - Per-object comparison across evals

### 4. Metrics JSON
`analysis/metrics.json` contains all metrics in structured format:

```json
{
  "experiment_id": "my_experiment",
  "analyzed_at": "2026-02-05T10:30:00",
  "training": {
    "overall": {
      "total_attempts": 1500,
      "successes": 1200,
      "success_rate": 0.80,
      "avg_confidence": 0.75
    },
    "per_object": { ... }
  },
  "evaluations": {
    "seen": {
      "overall": { ... },
      "per_object": { ... }
    },
    "unseen_simple": { ... },
    "unseen_complex": { ... }
  }
}
```

## Troubleshooting

### Experiment not found
```bash
# Check if experiment exists
python experiment_cli.py list

# Check experiment details
python experiment_cli.py info --experiment my_exp
```

### Eval phase not defined
```bash
# List available eval phases
python experiment_cli.py list-evals --experiment my_exp
```

### Path issues in container
```bash
# Check environment detection
python -c "from tossingbot.utils.container_paths import get_resolver; get_resolver().print_environment_info()"
```

### Training not completing
Training runs indefinitely until you stop it (Ctrl+C). The `steps` field in config is just a target reference.

## Best Practices

1. **Name eval phases semantically**: Use descriptive names like `seen`, `unseen_simple`, `generalization` rather than `eval1`, `eval2`

2. **Start small**: Test with short training (500 steps) and few episodes (50) before full experiments

3. **Use version numbers**: Include version in experiment names (e.g., `exp_baseline_v1`, `exp_baseline_v2`)

4. **Document descriptions**: Add clear descriptions to each eval phase explaining what it tests

5. **Compare incrementally**: Add eval phases one at a time to see how each affects generalization

6. **Archive old experiments**: Use `archive` command to keep experiment list clean

## Migration from Old System

To migrate an old session to new experiment format:

```bash
# TODO: Migration tool will be provided
python scripts/migrate_sessions.py --session old_session_id
```

## Tips

- **Reuse checkpoints**: You can run new eval phases on existing training without retraining
- **Compare checkpoints**: Use different `checkpoint` values in eval phases to compare training progression
- **Mix object sets**: Create eval phases with mixed seen/unseen objects to test robustness
- **Parallel evaluation**: Run multiple eval phases in parallel if you have multiple GPUs
- **Container persistence**: Always mount `sessions/` directory to persist results

## Support

For issues or questions:
1. Check this documentation
2. Review example configs in `experiments/`
3. Run with `--help` flag for command-specific help
4. Check experiment status with `info` command
