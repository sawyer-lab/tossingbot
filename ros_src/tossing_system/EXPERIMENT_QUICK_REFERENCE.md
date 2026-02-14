# Multi-Eval Experiment System - Quick Reference

## One-Page Cheat Sheet

### Core Concept
```
1 Experiment = 1 Training + N Evaluations

experiment/
├── train/              ← Train once
├── evals/
│   ├── seen/          ← Eval many times
│   ├── unseen/        ← Different object sets
│   └── mixed/         ← Different checkpoints
└── analysis/          ← Unified comparison
```

### Quick Start (3 Steps)

```bash
# 1. Create config
cat > experiments/my_exp.yaml <<EOF
name: my_exp
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
EOF

# 2. Run experiment
python src/tossingbot/scripts/experiment_cli.py run \
  --experiment my_exp --phase all

# 3. View results
python src/tossingbot/scripts/experiment_cli.py info \
  --experiment my_exp
```

### Common Commands

```bash
# Create
python experiment_cli.py create --config experiments/my_exp.yaml

# List
python experiment_cli.py list
python experiment_cli.py list --status completed

# Info
python experiment_cli.py info --experiment my_exp
python experiment_cli.py list-evals --experiment my_exp

# Run
python experiment_cli.py run --experiment my_exp --phase train
python experiment_cli.py run --experiment my_exp --phase eval --eval-name seen
python experiment_cli.py run --experiment my_exp --phase eval --eval-name all
python experiment_cli.py run --experiment my_exp --phase all

# Analyze
python experiment_cli.py analyze --experiment my_exptossing_system/src/tossingbot

# Compare
python experiment_cli.py compare --experiments exp1 exp2

# Manage
python experiment_cli.py archive --experiment old_exp
python experiment_cli.py delete --experiment bad_exp --force
```

### Config Template

```yaml
name: <experiment_name>
description: "Brief description"

train:
  objects: [obj1, obj2, obj3]  # Training objects
  steps: 1500                   # Target steps

evals:
  <eval_name_1>:
    description: "What this tests"
    objects: [obj1, obj2]        # Objects for this eval
    episodes: 100                # Number of episodes
    checkpoint: best             # best, latest, or step_N

  <eval_name_2>:
    # Add more eval phases...
```

### Object Names
```
Simple:    cube, cylinder, sphere, bar, puck, bolt
Complex:   L_shape, I_shape, T_shape, C_shape, cross, H_shape
```

### Checkpoint Options
```
best      - Best performing checkpoint (recommended)
latest    - Most recent checkpoint
step_500  - Checkpoint at step 500
step_1000 - Checkpoint at step 1000
```

### Directory Structure
```
tossing_system/
├── experiments/               # YAML configs
│   └── *.yaml
└── sessions/
    └── experiments/           # Experiment data
        └── <exp_id>/
            ├── train/         # Training artifacts
            ├── evals/         # Multiple eval phases
            └── analysis/      # Unified analysis
```

### Workflow Patterns

**Pattern 1: Full Auto**
```bash
python experiment_cli.py run --experiment my_exp --phase all
# Runs train → all evals → analyze
```

**Pattern 2: Manual Steps**
```bash
python experiment_cli.py run --experiment my_exp --phase train
python experiment_cli.py run --experiment my_exp --phase eval --eval-name all
python experiment_cli.py analyze --experiment my_exp
```

**Pattern 3: Incremental Eval**
```bash
python experiment_cli.py run --experiment my_exp --phase train
python experiment_cli.py run --experiment my_exp --phase eval --eval-name seen
python experiment_cli.py run --experiment my_exp --phase eval --eval-name unseen
python experiment_cli.py analyze --experiment my_exp
```

**Pattern 4: Add Eval Later**
```bash
# Edit config, add new eval phase
vim experiments/my_exp.yaml

# Run only new eval (no retraining!)
python experiment_cli.py run --experiment my_exp --phase eval --eval-name new_eval

# Re-analyze with new eval
python experiment_cli.py analyze --experiment my_exp
```

### Analysis Outputs
```
sessions/experiments/<exp_id>/analysis/
├── train_plots/                      # Training phase
│   ├── success_rate.png
│   ├── loss_curve.png
│   ├── per_object_success.png
│   └── ...
├── eval_<name>_plots/                # Each eval phase
│   ├── per_object_success.png
│   ├── confidence_distribution.png
│   └── ...
├── comparison_plots/                 # Cross-eval comparison
│   ├── eval_comparison_success.png
│   ├── eval_comparison_attempts.png
│   └── eval_comparison_per_object.png
├── metrics.json                      # All metrics
└── summary.txt                       # Text summary
```

### Example Configs

**Baseline (4 evals)**
```yaml
experiments/exp_multi_eval_baseline.yaml
# seen, unseen_simple, unseen_complex, mixed
```

**Progressive (5 evals)**
```yaml
experiments/exp_progressive_generalization.yaml
# seen → unseen simple → moderate → complex → all
```

**Checkpoint Comparison (5 evals)**
```yaml
experiments/exp_checkpoint_comparison.yaml
# early, mid, late, best, final
```

**Simple Test (2 evals)**
```yaml
experiments/exp_simple_test.yaml
# seen, unseen (quick test)
```

### Container Usage

```bash
# Run in container
docker run --rm -it \
  -v $(pwd)/sessions:/workspace/sessions \
  -v $(pwd)/experiments:/workspace/experiments \
  tossingbot:latest \
  python experiment_cli.py run --experiment my_exp --phase all

# Results persist in ./sessions/experiments/my_exp/
```

### Troubleshooting

**"Experiment not found"**
```bash
python experiment_cli.py list
```

**"Eval phase not defined"**
```bash
python experiment_cli.py list-evals --experiment my_exp
```

**Check environment**
```bash
python -c "from tossingbot.utils.container_paths import get_resolver; \
           get_resolver().print_environment_info()"
```

**Path issues**
```bash
# Check paths
python experiment_cli.py info --experiment my_exp
ls -la sessions/experiments/my_exp/
```

### Tips

✅ **Name eval phases semantically**: `seen`, `unseen_simple` not `eval1`, `eval2`

✅ **Start small**: Test with 500 steps, 50 episodes first

✅ **Use versions**: `exp_baseline_v1`, `exp_baseline_v2`

✅ **Add descriptions**: Explain what each eval tests

✅ **Reuse checkpoints**: Add new evals without retraining

✅ **Archive old**: Keep list clean with `archive` command

### Old vs New System

**Old System:**
```bash
# Train
auto_grasp.py --mode training --session X --train-objects A B C

# Eval (creates separate session)
auto_grasp.py --eval-only --session X --eval-objects D E F

# Analyze separately
session_tools.py analyze --session X
session_tools.py analyze --session X_eval
```

**New System:**
```bash
# Everything in one config
cat > experiments/X.yaml <<EOF
train:
  objects: [A, B, C]
evals:
  eval1: { objects: [A, B, C] }
  eval2: { objects: [D, E, F] }
EOF

# Run everything
experiment_cli.py run --experiment X --phase all

# Unified analysis
experiment_cli.py analyze --experiment X
```

### Help Commands

```bash
# General help
python experiment_cli.py --help

# Command-specific help
python experiment_cli.py run --help
python experiment_cli.py create --help
python experiment_cli.py analyze --help
```

### Documentation

- **Full Guide**: `EXPERIMENTS_README.md`
- **Implementation**: `IMPLEMENTATION_SUMMARY.md`
- **This File**: `EXPERIMENT_QUICK_REFERENCE.md`

---

**Key Innovation**: Train once, evaluate many times with automatic comparison! 🚀
