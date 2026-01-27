#!/usr/bin/env python3.8
"""
Run complete train/test experiment from YAML config.
Creates session, trains, evaluates, generates report.
"""
import sys
import os
import yaml
import argparse
import subprocess

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot import config as cfg


def load_experiment_config(yaml_path):
    """Load YAML experiment configuration"""
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


def run_experiment(config, workspace_path="/workspace/src/tossing_system"):
    """
    Execute complete experiment:
    1. Train model on specified objects
    2. Evaluate on test objects (if different)
    3. Generate analysis report
    
    Args:
        config: Experiment configuration dict
        workspace_path: Path to tossing_system directory
    """
    name = config['name']
    description = config.get('description', '')
    
    # Training configuration
    train_config = config['training']
    train_objects = train_config['objects']
    train_steps = train_config.get('steps', 1500)
    train_hyperparams = train_config.get('hyperparameters', {})
    
    # Evaluation configuration
    eval_config = config.get('evaluation', {})
    eval_objects = eval_config.get('objects', train_objects)
    eval_episodes = eval_config.get('episodes', 100)
    
    print(f"\n{'='*70}")
    print(f"EXPERIMENT: {name}")
    print(f"{'='*70}")
    print(f"Description: {description}")
    print(f"Train on: {', '.join(train_objects)}")
    print(f"Test on: {', '.join(eval_objects)}")
    print(f"Training steps: {train_steps}")
    print(f"Evaluation episodes: {eval_episodes}")
    print(f"{'='*70}\n")
    
    # Phase 1: Training
    print(f"\n{'='*70}")
    print("[1/3] TRAINING PHASE")
    print(f"{'='*70}\n")
    
    train_cmd = [
        'python3.8', os.path.join(workspace_path, 'src/tossingbot/scripts/auto_grasp.py'),
        '--mode', 'training',
        '--session', name,
        '--train-objects', *train_objects,
    ]
    
    print(f"Command: {' '.join(train_cmd)}")
    print(f"\nPress Ctrl+C after ~{train_steps} steps to stop training...\n")
    
    try:
        subprocess.run(train_cmd)
    except KeyboardInterrupt:
        print("\nTraining stopped by user.")
    
    # Phase 2: Evaluation (if test objects different from train)
    if set(eval_objects) != set(train_objects):
        print(f"\n{'='*70}")
        print("[2/3] EVALUATION PHASE")
        print(f"{'='*70}\n")
        
        eval_cmd = [
            'python3.8', os.path.join(workspace_path, 'src/tossingbot/scripts/auto_grasp.py'),
            '--eval-only',
            '--session', name,
            '--eval-objects', *eval_objects,
            '--eval-episodes', str(eval_episodes),
        ]
        
        print(f"Command: {' '.join(eval_cmd)}")
        print()
        
        try:
            subprocess.run(eval_cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Evaluation failed: {e}")
            return False
        except KeyboardInterrupt:
            print("\nEvaluation stopped by user.")
            return False
    else:
        print(f"\n[2/3] EVALUATION PHASE - SKIPPED (train==test objects)")
    
    # Phase 3: Generate Report
    print(f"\n{'='*70}")
    print("[3/3] GENERATING REPORT")
    print(f"{'='*70}\n")
    
    report_cmd = [
        'python3.8', os.path.join(workspace_path, 'src/tossingbot/scripts/evaluate_generalization.py'),
        '--session', name,
    ]
    
    try:
        subprocess.run(report_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Report generation failed: {e}")
        return False
    
    print(f"\n{'='*70}")
    print(f"EXPERIMENT COMPLETE: {name}")
    print(f"{'='*70}")
    print(f"Session directory: sessions/training/{name}/")
    print(f"View results:")
    print(f"  - Analyze: ./analyze_session.sh {name}")
    print(f"  - Visualize: ./visualize_object_grasps.sh {name}")
    print(f"{'='*70}\n")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Run TossingBot train/test experiment from YAML config',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Example config file (experiments/exp_baseline.yaml):
  
  name: exp_baseline
  description: "Baseline: train and test on same objects"
  training:
    objects: [L_shape, I_shape, T_shape]
    steps: 1500
  evaluation:
    objects: [L_shape, I_shape, T_shape]
    episodes: 100

Run with:
  python run_experiment.py experiments/exp_baseline.yaml
        '''
    )
    parser.add_argument('config', help='Path to experiment YAML config file')
    parser.add_argument('--workspace', default='/workspace/src/tossing_system',
                        help='Path to tossing_system workspace')
    
    args = parser.parse_args()
    
    # Load config
    try:
        config = load_experiment_config(args.config)
    except Exception as e:
        print(f"Error loading config: {e}")
        sys.exit(1)
    
    # Run experiment
    success = run_experiment(config, args.workspace)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
