#!/usr/bin/env python
"""
Generate presentation-ready figures from experiment logs.

Usage:
    python generate_presentation_figures.py final_small --output ~/slides/figures/
"""

import os
import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path

# Use non-interactive backend for saving figures
matplotlib.use('Agg')

# Set publication-quality defaults
plt.rcParams.update({
    'font.size': 14,
    'axes.labelsize': 16,
    'axes.titlesize': 18,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
    'figure.titlesize': 20,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'font.family': 'serif',
})

def load_jsonl_log(log_path):
    """Load a JSONL log file and return list of entries."""
    entries = []
    if not os.path.exists(log_path):
        print(f"Warning: Log file not found: {log_path}")
        return entries
    
    with open(log_path, 'r') as f:
        for line in f:
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return entries

def compute_rolling_average(values, window=50):
    """Compute rolling average with given window size."""
    if len(values) < window:
        return np.array(values)
    
    rolling = np.convolve(values, np.ones(window)/window, mode='valid')
    # Pad beginning to match original length
    pad_length = len(values) - len(rolling)
    return np.concatenate([np.full(pad_length, np.nan), rolling])

def plot_training_convergence(train_log, output_dir):
    """Generate training convergence plot (success rate + loss)."""
    print("Generating training convergence plot...")
    
    steps = [entry['step'] for entry in train_log]
    successes = [1 if entry['success'] else 0 for entry in train_log]
    losses = [entry['loss'] for entry in train_log]
    
    # Compute rolling average
    success_rolling = compute_rolling_average(successes, window=50)
    
    # Create figure with two y-axes
    fig, ax1 = plt.subplots(figsize=(12, 6))
    
    # Plot success rate
    color1 = 'tab:blue'
    ax1.set_xlabel('Training Steps')
    ax1.set_ylabel('Success Rate', color=color1)
    ax1.plot(steps, successes, 'o', alpha=0.1, markersize=3, color=color1, label='Per-step')
    ax1.plot(steps, success_rolling, '-', linewidth=2.5, color=color1, label='Rolling avg (50 steps)')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.set_ylim([-0.05, 1.05])
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left')
    
    # Plot loss on secondary axis
    ax2 = ax1.twinx()
    color2 = 'tab:red'
    ax2.set_ylabel('Loss', color=color2)
    loss_rolling = compute_rolling_average(losses, window=50)
    ax2.plot(steps, loss_rolling, '-', linewidth=2, color=color2, alpha=0.7, label='Loss (50-step avg)')
    ax2.tick_params(axis='y', labelcolor=color2)
    ax2.legend(loc='upper right')
    
    plt.title('Training Convergence')
    fig.tight_layout()
    
    output_path = os.path.join(output_dir, 'training_convergence.pdf')
    plt.savefig(output_path, format='pdf')
    plt.savefig(output_path.replace('.pdf', '.png'), format='png')
    print(f"Saved: {output_path}")
    plt.close()

def compute_success_rate(log_entries):
    """Compute overall success rate from log entries."""
    if not log_entries:
        return 0.0
    successes = sum(1 for entry in log_entries if entry.get('success', False))
    return successes / len(log_entries)

def compute_per_object_success(log_entries, train_objects=None):
    """Compute success rate per object type."""
    object_stats = {}
    
    for entry in log_entries:
        obj_name = entry.get('object_name', 'unknown')
        if obj_name == 'unknown':
            continue
        
        # Strip instance numbers (e.g., "C_shape_small_0" -> "C_shape_small")
        base_name = obj_name.rsplit('_', 1)[0] if obj_name[-1].isdigit() else obj_name
        
        if base_name not in object_stats:
            object_stats[base_name] = {'success': 0, 'total': 0}
        
        object_stats[base_name]['total'] += 1
        if entry.get('success', False):
            object_stats[base_name]['success'] += 1
    
    # Compute success rates
    results = {}
    for obj, stats in object_stats.items():
        rate = stats['success'] / stats['total'] if stats['total'] > 0 else 0.0
        results[obj] = {
            'success_rate': rate,
            'successes': stats['success'],
            'total': stats['total'],
            'is_training': train_objects and obj in train_objects
        }
    
    return results

def plot_seen_vs_unseen(seen_log, unseen_log, train_objects, output_dir):
    """Generate bar chart comparing seen vs unseen performance."""
    print("Generating seen vs unseen comparison...")
    
    seen_rate = compute_success_rate(seen_log)
    unseen_rate = compute_success_rate(unseen_log)
    
    # Create bar chart
    fig, ax = plt.subplots(figsize=(8, 6))
    
    categories = ['Training Objects\n(Seen)', 'Novel Objects\n(Unseen)']
    rates = [seen_rate * 100, unseen_rate * 100]
    colors = ['#2E86AB', '#A23B72']
    
    bars = ax.bar(categories, rates, color=colors, width=0.6, edgecolor='black', linewidth=1.5)
    
    # Add value labels on bars
    for bar, rate in zip(bars, rates):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 2,
                f'{rate:.1f}%',
                ha='center', va='bottom', fontsize=16, fontweight='bold')
    
    ax.set_ylabel('Success Rate (%)')
    ax.set_ylim([0, 100])
    ax.set_title('Grasping Generalization Performance')
    ax.grid(axis='y', alpha=0.3)
    
    # Add horizontal line at 50% for reference
    ax.axhline(y=50, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    
    output_path = os.path.join(output_dir, 'seen_vs_unseen_comparison.pdf')
    plt.savefig(output_path, format='pdf')
    plt.savefig(output_path.replace('.pdf', '.png'), format='png')
    print(f"Saved: {output_path}")
    plt.close()

def plot_per_object_breakdown(seen_log, unseen_log, train_objects, output_dir):
    """Generate bar chart with per-object success rates."""
    print("Generating per-object breakdown...")
    
    # Compute per-object stats
    seen_stats = compute_per_object_success(seen_log, train_objects)
    unseen_stats = compute_per_object_success(unseen_log, train_objects)
    
    # Combine all objects
    all_stats = {**seen_stats, **unseen_stats}
    
    # Sort by success rate
    sorted_objects = sorted(all_stats.items(), key=lambda x: x[1]['success_rate'], reverse=True)
    
    objects = [obj for obj, _ in sorted_objects]
    rates = [stats['success_rate'] * 100 for _, stats in sorted_objects]
    colors = ['#2E86AB' if stats['is_training'] else '#A23B72' for _, stats in sorted_objects]
    
    # Create bar chart
    fig, ax = plt.subplots(figsize=(12, 6))
    
    bars = ax.bar(range(len(objects)), rates, color=colors, edgecolor='black', linewidth=1.5)
    
    # Add value labels
    for i, (bar, rate, (obj, stats)) in enumerate(zip(bars, rates, sorted_objects)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 2,
                f'{rate:.1f}%',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
        # Add count below
        ax.text(bar.get_x() + bar.get_width()/2., -5,
                f'({stats["successes"]}/{stats["total"]})',
                ha='center', va='top', fontsize=9)
    
    ax.set_ylabel('Success Rate (%)')
    ax.set_ylim([0, 105])
    ax.set_xticks(range(len(objects)))
    ax.set_xticklabels([obj.replace('_small', '').replace('_', ' ').title() for obj in objects],
                        rotation=45, ha='right')
    ax.set_title('Per-Object Grasping Success Rate')
    ax.grid(axis='y', alpha=0.3)
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#2E86AB', edgecolor='black', label='Training Objects'),
        Patch(facecolor='#A23B72', edgecolor='black', label='Novel Objects')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    output_path = os.path.join(output_dir, 'per_object_breakdown.pdf')
    plt.savefig(output_path, format='pdf')
    plt.savefig(output_path.replace('.pdf', '.png'), format='png')
    print(f"Saved: {output_path}")
    plt.close()

def print_summary_stats(train_log, seen_log, unseen_log, train_objects):
    """Print summary statistics to console."""
    print("\n" + "="*70)
    print("SUMMARY STATISTICS")
    print("="*70)
    
    # Training stats
    train_rate = compute_success_rate(train_log)
    print(f"\nTraining Performance:")
    print(f"  Total steps: {len(train_log)}")
    print(f"  Overall success rate: {train_rate*100:.1f}%")
    
    # Last 500 steps
    if len(train_log) >= 500:
        last_500_rate = compute_success_rate(train_log[-500:])
        print(f"  Last 500 steps: {last_500_rate*100:.1f}%")
    
    # Evaluation stats
    seen_rate = compute_success_rate(seen_log)
    unseen_rate = compute_success_rate(unseen_log)
    
    print(f"\nEvaluation Performance:")
    print(f"  Seen objects: {seen_rate*100:.1f}% ({sum(1 for e in seen_log if e['success'])}/{len(seen_log)} episodes)")
    print(f"  Unseen objects: {unseen_rate*100:.1f}% ({sum(1 for e in unseen_log if e['success'])}/{len(unseen_log)} episodes)")
    print(f"  Generalization gap: {(seen_rate - unseen_rate)*100:.1f} percentage points")
    
    # Per-object breakdown
    print(f"\nPer-Object Performance (Unseen):")
    unseen_stats = compute_per_object_success(unseen_log, train_objects)
    for obj, stats in sorted(unseen_stats.items(), key=lambda x: x[1]['success_rate'], reverse=True):
        print(f"  {obj:20s}: {stats['success_rate']*100:5.1f}% ({stats['successes']:2d}/{stats['total']:2d})")
    
    print("\n" + "="*70 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Generate presentation figures from experiment logs')
    parser.add_argument('experiment_id', help='Experiment ID (e.g., final_small)')
    parser.add_argument('--output', default='./presentation_figures', help='Output directory for figures')
    parser.add_argument('--base-dir', default=None, help='Base directory for experiments (auto-detect if not provided)')
    
    args = parser.parse_args()
    
    # Auto-detect base directory
    if args.base_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.base_dir = os.path.join(script_dir, '../../sessions/experiments')
    
    # Construct paths
    exp_dir = os.path.join(args.base_dir, args.experiment_id)
    if not os.path.exists(exp_dir):
        print(f"Error: Experiment directory not found: {exp_dir}")
        return 1
    
    train_log_path = os.path.join(exp_dir, 'train/logs/training_log.jsonl')
    seen_log_path = os.path.join(exp_dir, 'evals/seen/logs/evaluation_log.jsonl')
    unseen_log_path = os.path.join(exp_dir, 'evals/unseen/logs/evaluation_log.jsonl')
    
    # Load config to get training objects
    config_path = os.path.join(exp_dir, 'config.yaml')
    train_objects = None
    if os.path.exists(config_path):
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            train_objects = config.get('train', {}).get('objects', [])
    
    print(f"\nLoading logs from experiment: {args.experiment_id}")
    print(f"Experiment directory: {exp_dir}")
    
    # Load logs
    train_log = load_jsonl_log(train_log_path)
    seen_log = load_jsonl_log(seen_log_path)
    unseen_log = load_jsonl_log(unseen_log_path)
    
    print(f"  Training log: {len(train_log)} entries")
    print(f"  Seen eval log: {len(seen_log)} entries")
    print(f"  Unseen eval log: {len(unseen_log)} entries")
    
    if not train_log:
        print("Error: No training data found!")
        return 1
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    print(f"\nOutput directory: {args.output}")
    
    # Generate figures
    print("\nGenerating figures...")
    plot_training_convergence(train_log, args.output)
    
    if seen_log and unseen_log:
        plot_seen_vs_unseen(seen_log, unseen_log, train_objects, args.output)
        plot_per_object_breakdown(seen_log, unseen_log, train_objects, args.output)
        print_summary_stats(train_log, seen_log, unseen_log, train_objects)
    else:
        print("\nWarning: Evaluation logs not found, skipping comparison plots")
    
    print(f"\n✓ All figures generated successfully in: {args.output}")
    return 0

if __name__ == '__main__':
    sys.exit(main())
