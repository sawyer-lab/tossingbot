#!/usr/bin/env python3.8
"""
Unified Experiment Analysis for TossingBot
Analyzes training + multiple evaluation phases together
Generates comparison plots and unified reports
"""
import os
import sys
import json
from collections import defaultdict
from typing import List, Dict, Any, Optional
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot.learning.experiment_session import ExperimentSession
from tossingbot.scripts import analyze_training
from tossingbot.scripts import visualize_object_grasps


def load_log(log_path):
    """Load training/eval log from JSONL file"""
    if not os.path.exists(log_path):
        return []

    entries = []
    with open(log_path, 'r') as f:
        for line in f:
            if line.strip():
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return entries


def extract_steps(entries):
    """Extract only step entries (filter out episode markers)"""
    return [e for e in entries if e.get('event_type') != 'episode_end']


def compute_last_n_success_rate(steps, n=50):
    """
    Compute success rate over the last N attempts.
    
    Args:
        steps: List of step entries
        n: Number of recent steps to consider
    
    Returns:
        Success rate (0.0 to 1.0), or None if insufficient data
    """
    if len(steps) < n:
        if len(steps) == 0:
            return None
        n = len(steps)
    
    recent_steps = steps[-n:]
    successes = sum(1 for s in recent_steps if s.get('success', False))
    return successes / n


def compute_metrics(steps):
    """Compute metrics from step entries"""
    if not steps:
        return {
            'total_attempts': 0,
            'successes': 0,
            'success_rate': 0.0,
            'avg_confidence': 0.0,
            'last_50_success_rate': None
        }

    total = len(steps)
    successes = sum(1 for s in steps if s.get('success', False))
    confidences = [s.get('predicted_grasp', {}).get('confidence', 0.0) for s in steps]

    return {
        'total_attempts': total,
        'successes': successes,
        'success_rate': successes / total if total > 0 else 0.0,
        'avg_confidence': np.mean(confidences) if confidences else 0.0,
        'last_50_success_rate': compute_last_n_success_rate(steps, n=50)
    }


def compute_per_object_metrics(steps):
    """Compute per-object statistics"""
    object_stats = defaultdict(lambda: {'attempts': 0, 'successes': 0})

    for step in steps:
        obj_name = step.get('object_name', 'unknown')
        if obj_name != 'unknown':
            object_stats[obj_name]['attempts'] += 1
            if step.get('success', False):
                object_stats[obj_name]['successes'] += 1

    # Calculate success rates
    for obj_name in object_stats:
        stats = object_stats[obj_name]
        stats['success_rate'] = stats['successes'] / stats['attempts'] if stats['attempts'] > 0 else 0.0

    return dict(object_stats)


def generate_eval_comparison_plots(eval_logs: Dict[str, List], output_dir: str):
    """
    Generate comparison plots across multiple eval phases

    Args:
        eval_logs: Dictionary mapping eval_name to list of log entries
        output_dir: Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)

    eval_names = list(eval_logs.keys())
    if len(eval_names) == 0:
        return

    # Extract steps for each eval
    eval_steps = {name: extract_steps(log) for name, log in eval_logs.items()}

    # Compute metrics for each eval
    eval_metrics = {name: compute_metrics(steps) for name, steps in eval_steps.items()}

    # ===== 1. Success Rate Comparison =====
    fig, ax = plt.subplots(figsize=(12, 6))

    success_rates = [eval_metrics[name]['success_rate'] for name in eval_names]

    bars = ax.bar(eval_names, success_rates, color='steelblue', alpha=0.8)
    ax.set_ylabel('Success Rate', fontsize=12)
    ax.set_title('Success Rate Across Evaluation Phases', fontsize=14, fontweight='bold')
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3, axis='y')
    plt.xticks(rotation=45, ha='right')

    # Add percentage labels on bars
    for i, (bar, rate) in enumerate(zip(bars, success_rates)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{rate*100:.1f}%',
                ha='center', va='bottom', fontsize=10, fontweight='bold')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'eval_comparison_success.png'), dpi=150)
    plt.close()

    # ===== 2. Attempt Count Comparison =====
    fig, ax = plt.subplots(figsize=(12, 6))

    attempt_counts = [eval_metrics[name]['total_attempts'] for name in eval_names]

    bars = ax.bar(eval_names, attempt_counts, color='coral', alpha=0.8)
    ax.set_ylabel('Number of Attempts', fontsize=12)
    ax.set_title('Attempts per Evaluation Phase', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    plt.xticks(rotation=45, ha='right')

    # Add count labels
    for i, (bar, count) in enumerate(zip(bars, attempt_counts)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{count}',
                ha='center', va='bottom', fontsize=10)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'eval_comparison_attempts.png'), dpi=150)
    plt.close()

    # ===== 3. Per-Object Performance Across Evals =====
    # Collect all unique objects across all evals
    all_objects = set()
    per_eval_obj_metrics = {}

    for eval_name, steps in eval_steps.items():
        obj_metrics = compute_per_object_metrics(steps)
        per_eval_obj_metrics[eval_name] = obj_metrics
        all_objects.update(obj_metrics.keys())

    if all_objects:
        all_objects = sorted(list(all_objects))

        fig, ax = plt.subplots(figsize=(14, 8))

        # Grouped bar chart
        x = np.arange(len(all_objects))
        width = 0.8 / len(eval_names)

        for i, eval_name in enumerate(eval_names):
            obj_metrics = per_eval_obj_metrics[eval_name]
            rates = [obj_metrics.get(obj, {}).get('success_rate', 0.0) for obj in all_objects]

            offset = (i - len(eval_names)/2 + 0.5) * width
            ax.bar(x + offset, rates, width, label=eval_name, alpha=0.8)

        ax.set_ylabel('Success Rate', fontsize=12)
        ax.set_title('Per-Object Performance Across Evaluation Phases', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(all_objects, rotation=45, ha='right')
        ax.set_ylim(0, 1.05)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'eval_comparison_per_object.png'), dpi=150)
        plt.close()

    print(f"  Generated evaluation comparison plots in: {output_dir}")


def analyze_experiment(experiment: ExperimentSession, 
                      eval_names: Optional[List[str]] = None,
                      include_object_grasps: bool = True) -> bool:
    """
    Generate unified analysis combining training and multiple evaluations

    Args:
        experiment: ExperimentSession object
        eval_names: List of eval phase names to analyze, or None for all
        include_object_grasps: Whether to generate per-object grasp visualizations

    Returns:
        True if successful, False otherwise
    """
    print(f"\n{'='*70}")
    print(f"ANALYZING EXPERIMENT: {experiment.experiment_id}")
    print(f"{'='*70}\n")

    # ===== 1. Training Analysis =====
    train_log_path = experiment.get_train_log_path()

    if os.path.exists(train_log_path):
        print("Analyzing training phase...")
        train_output_dir = experiment.get_analysis_plots_dir("train_plots")
        success = analyze_training.analyze_session_from_path(train_log_path, train_output_dir, window=50)

        if not success:
            print("Warning: Training analysis failed")
        
        # Per-object grasp visualizations for training
        if include_object_grasps:
            print("  Generating per-object grasp visualizations for training...")
            try:
                visualize_object_grasps.analyze_per_object_grasps(train_log_path, train_output_dir)
            except Exception as e:
                print(f"  Warning: Per-object grasp visualization failed: {e}")
    else:
        print("Warning: No training log found. Skipping training analysis.")

    # ===== 2. Evaluation Analysis =====
    # Determine which eval phases to analyze
    if eval_names is None:
        eval_names = experiment.list_eval_phases()

    if not eval_names:
        print("\nNo evaluation phases to analyze.")
        print(f"\n{'='*70}")
        print(f"Analysis complete!")
        print(f"Results saved to: {experiment.analysis_dir}")
        print(f"{'='*70}\n")
        return True

    print(f"\nAnalyzing {len(eval_names)} evaluation phase(s)...")

    # Load all eval logs
    eval_logs = {}
    eval_metrics = {}

    for eval_name in eval_names:
        eval_log_path = experiment.get_eval_log_path(eval_name)

        if os.path.exists(eval_log_path):
            print(f"  Loading eval phase: {eval_name}")
            eval_logs[eval_name] = load_log(eval_log_path)

            # Compute metrics
            steps = extract_steps(eval_logs[eval_name])
            eval_metrics[eval_name] = {
                'overall': compute_metrics(steps),
                'per_object': compute_per_object_metrics(steps)
            }

            # Generate per-eval plots (reuse existing training analysis functions)
            eval_output_dir = experiment.get_analysis_plots_dir(f"eval_{eval_name}_plots")
            os.makedirs(eval_output_dir, exist_ok=True)

            # Generate basic plots for this eval
            if steps:
                # Success rate over time
                analyze_training.plot_per_object_success(steps, eval_output_dir)
                analyze_training.plot_confidence_distribution(steps, eval_output_dir)
                analyze_training.plot_grasp_heatmap(steps, eval_output_dir)
                analyze_training.plot_rotation_distribution(steps, eval_output_dir)
                
                # Per-object grasp visualizations
                if include_object_grasps:
                    print(f"    Generating per-object grasp visualizations for {eval_name}...")
                    try:
                        visualize_object_grasps.analyze_per_object_grasps(eval_log_path, eval_output_dir)
                    except Exception as e:
                        print(f"    Warning: Per-object grasp visualization failed: {e}")
        else:
            print(f"  Warning: No log found for eval phase '{eval_name}'")

    # ===== 3. Generate Comparison Plots =====
    if len(eval_logs) > 1:
        print("\nGenerating cross-evaluation comparison plots...")
        comparison_dir = experiment.get_analysis_plots_dir("comparison_plots")
        generate_eval_comparison_plots(eval_logs, comparison_dir)
    elif len(eval_logs) == 1:
        print("\nOnly one eval phase analyzed. Skipping comparison plots.")

    # ===== 4. Save Unified Metrics JSON =====
    print("\nSaving unified metrics...")

    # Load training metrics if available
    train_metrics = {}
    if os.path.exists(train_log_path):
        train_steps = extract_steps(load_log(train_log_path))
        if train_steps:
            train_metrics = {
                'overall': compute_metrics(train_steps),
                'per_object': compute_per_object_metrics(train_steps)
            }

    unified_metrics = {
        'experiment_id': experiment.experiment_id,
        'analyzed_at': __import__('datetime').datetime.now().isoformat(),
        'training': train_metrics,
        'evaluations': eval_metrics
    }

    metrics_path = os.path.join(experiment.analysis_dir, 'metrics.json')
    with open(metrics_path, 'w') as f:
        json.dump(unified_metrics, f, indent=2)

    print(f"  Saved metrics to: {metrics_path}")

    # ===== 5. Generate Text Summary =====
    summary_path = os.path.join(experiment.analysis_dir, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write(f"EXPERIMENT ANALYSIS SUMMARY\n")
        f.write(f"Experiment: {experiment.experiment_id}\n")
        f.write("=" * 70 + "\n\n")

        # Training summary
        if train_metrics:
            f.write("TRAINING PHASE:\n")
            f.write("-" * 70 + "\n")
            overall = train_metrics['overall']
            f.write(f"  Total Steps: {overall['total_attempts']}\n")
            f.write(f"  Successes: {overall['successes']}\n")
            f.write(f"  Success Rate: {overall['success_rate']*100:.2f}%\n")
            if overall.get('last_50_success_rate') is not None:
                f.write(f"  Last 50 Attempts: {overall['last_50_success_rate']*100:.2f}%\n")
            f.write(f"  Avg Confidence: {overall['avg_confidence']:.4f}\n\n")

        # Evaluation summaries
        f.write("EVALUATION PHASES:\n")
        f.write("-" * 70 + "\n")
        for eval_name, metrics in eval_metrics.items():
            overall = metrics['overall']
            f.write(f"\n{eval_name}:\n")
            f.write(f"  Attempts: {overall['total_attempts']}\n")
            f.write(f"  Successes: {overall['successes']}\n")
            f.write(f"  Success Rate: {overall['success_rate']*100:.2f}%\n")
            if overall.get('last_50_success_rate') is not None:
                f.write(f"  Last 50 Attempts: {overall['last_50_success_rate']*100:.2f}%\n")
            f.write(f"  Avg Confidence: {overall['avg_confidence']:.4f}\n")

        f.write("\n" + "=" * 70 + "\n")

    print(f"  Saved summary to: {summary_path}")

    # ===== Done =====
    print(f"\n{'='*70}")
    print(f"Analysis complete!")
    print(f"Results saved to: {experiment.analysis_dir}")
    print(f"\nGenerated outputs:")
    print(f"  - Training plots: {experiment.get_analysis_plots_dir('train_plots')}")
    for eval_name in eval_logs.keys():
        print(f"  - Eval '{eval_name}' plots: {experiment.get_analysis_plots_dir(f'eval_{eval_name}_plots')}")
    if len(eval_logs) > 1:
        print(f"  - Comparison plots: {experiment.get_analysis_plots_dir('comparison_plots')}")
    print(f"  - Unified metrics: {metrics_path}")
    print(f"  - Summary: {summary_path}")
    print(f"{'='*70}\n")

    return True


if __name__ == '__main__':
    import argparse
    from tossingbot import config as cfg
    from tossingbot.learning.experiment_manager import ExperimentManager

    parser = argparse.ArgumentParser(description='Analyze TossingBot experiment')
    parser.add_argument('--experiment', required=True, help='Experiment ID')
    parser.add_argument('--eval-names', nargs='+', help='Specific eval phases to analyze')

    args = parser.parse_args()

    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")
    experiment = manager.load_experiment(args.experiment)

    if experiment is None:
        print(f"Error: Experiment not found: {args.experiment}")
        sys.exit(1)

    success = analyze_experiment(experiment, args.eval_names)
    sys.exit(0 if success else 1)
