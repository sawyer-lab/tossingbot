#!/usr/bin/env python3
"""
Training Analysis Script for TossingBot
Generates plots and statistics from training logs
"""
import argparse
import json
import os
import sys
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from datetime import datetime


def load_log(log_path):
    """Load training log from JSONL file"""
    entries = []
    with open(log_path, 'r') as f:
        for line in f:
            if line.strip():
                entries.append(json.loads(line))
    return entries


def extract_steps(entries):
    """Extract only step entries (filter out episode markers)"""
    return [e for e in entries if e.get('event_type') != 'episode_end']


def extract_episodes(entries):
    """Extract only episode end markers"""
    return [e for e in entries if e.get('event_type') == 'episode_end']


def moving_average(data, window_size):
    """Compute moving average"""
    if len(data) < window_size:
        return data
    cumsum = np.cumsum(np.insert(data, 0, 0))
    return (cumsum[window_size:] - cumsum[:-window_size]) / window_size


def plot_success_rate(steps, output_dir, window=50):
    """Plot success rate over time"""
    successes = [1 if s['success'] else 0 for s in steps]
    step_numbers = [s['step'] for s in steps]
    
    # Compute moving average
    ma_success = moving_average(successes, window)
    ma_steps = step_numbers[window-1:]
    
    plt.figure(figsize=(12, 6))
    plt.plot(step_numbers, successes, 'o', alpha=0.2, label='Individual attempts', markersize=3)
    plt.plot(ma_steps, ma_success, 'r-', linewidth=2, label=f'{window}-step moving average')
    plt.xlabel('Training Step')
    plt.ylabel('Success Rate')
    plt.title('Grasp Success Rate Over Time')
    plt.ylim(-0.05, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'success_rate.png'), dpi=150)
    plt.close()
    print(f"  Generated: success_rate.png")


def plot_per_object_success(steps, output_dir):
    """Plot success rate by object type"""
    object_stats = defaultdict(lambda: {'attempts': 0, 'successes': 0})
    
    for step in steps:
        obj_name = step.get('object_name', 'unknown')
        if obj_name != 'unknown':
            object_stats[obj_name]['attempts'] += 1
            if step['success']:
                object_stats[obj_name]['successes'] += 1
    
    # Calculate success rates
    objects = sorted(object_stats.keys())
    success_rates = []
    attempt_counts = []
    
    for obj in objects:
        stats = object_stats[obj]
        if stats['attempts'] > 0:
            success_rates.append(stats['successes'] / stats['attempts'])
            attempt_counts.append(stats['attempts'])
        else:
            success_rates.append(0)
            attempt_counts.append(0)
    
    # Create bar plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Success rates
    bars1 = ax1.bar(objects, success_rates, color='steelblue')
    ax1.set_ylabel('Success Rate')
    ax1.set_title('Success Rate by Object Type')
    ax1.set_ylim(0, 1.05)
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Add percentage labels on bars
    for bar, rate in zip(bars1, success_rates):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{rate*100:.1f}%',
                ha='center', va='bottom', fontsize=10)
    
    # Attempt counts
    bars2 = ax2.bar(objects, attempt_counts, color='coral')
    ax2.set_ylabel('Number of Attempts')
    ax2.set_xlabel('Object Type')
    ax2.set_title('Attempts per Object')
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Add count labels
    for bar, count in zip(bars2, attempt_counts):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{count}',
                ha='center', va='bottom', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'per_object_success.png'), dpi=150)
    plt.close()
    print(f"  Generated: per_object_success.png")


def plot_loss_curve(steps, output_dir, window=50):
    """Plot training loss over time"""
    losses = [s['loss'] for s in steps if s['loss'] > 0]
    step_numbers = [s['step'] for s in steps if s['loss'] > 0]
    
    if len(losses) == 0:
        print("  Warning: No loss data available (inference mode?)")
        return
    
    # Compute moving average
    ma_loss = moving_average(losses, min(window, len(losses)))
    ma_steps = step_numbers[min(window, len(losses))-1:]
    
    plt.figure(figsize=(12, 6))
    plt.plot(step_numbers, losses, alpha=0.3, label='Loss', linewidth=1)
    plt.plot(ma_steps, ma_loss, 'r-', linewidth=2, label=f'{window}-step moving average')
    plt.xlabel('Training Step')
    plt.ylabel('Loss')
    plt.title('Training Loss Over Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'loss_curve.png'), dpi=150)
    plt.close()
    print(f"  Generated: loss_curve.png")


def plot_epsilon_decay(steps, output_dir):
    """Plot epsilon (exploration rate) over time"""
    epsilons = [s['epsilon'] for s in steps]
    step_numbers = [s['step'] for s in steps]
    
    plt.figure(figsize=(12, 6))
    plt.plot(step_numbers, epsilons, 'g-', linewidth=2)
    plt.xlabel('Training Step')
    plt.ylabel('Epsilon (Exploration Rate)')
    plt.title('Exploration Rate Decay')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'epsilon_decay.png'), dpi=150)
    plt.close()
    print(f"  Generated: epsilon_decay.png")


def plot_confidence_distribution(steps, output_dir):
    """Plot distribution of network confidence scores"""
    confidences = [s['predicted_grasp']['confidence'] for s in steps]
    successes = [s['success'] for s in steps]
    
    success_conf = [c for c, s in zip(confidences, successes) if s]
    failure_conf = [c for c, s in zip(confidences, successes) if not s]
    
    plt.figure(figsize=(12, 6))
    plt.hist(failure_conf, bins=50, alpha=0.6, label='Failures', color='red')
    plt.hist(success_conf, bins=50, alpha=0.6, label='Successes', color='green')
    plt.xlabel('Network Confidence')
    plt.ylabel('Frequency')
    plt.title('Network Confidence Distribution (Success vs Failure)')
    plt.legend()
    plt.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'confidence_distribution.png'), dpi=150)
    plt.close()
    print(f"  Generated: confidence_distribution.png")


def plot_grasp_heatmap(steps, output_dir):
    """Plot heatmap of grasp locations"""
    # Collect grasp locations
    successes_u = []
    successes_v = []
    failures_u = []
    failures_v = []
    
    for step in steps:
        grasp = step['predicted_grasp']
        u, v = grasp['u'], grasp['v']
        if step['success']:
            successes_u.append(u)
            successes_v.append(v)
        else:
            failures_u.append(u)
            failures_v.append(v)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Successful grasps
    if successes_u:
        ax1.hexbin(successes_v, successes_u, gridsize=20, cmap='Greens', mincnt=1)
        ax1.set_title('Successful Grasp Locations')
        ax1.set_xlabel('v (pixel column)')
        ax1.set_ylabel('u (pixel row)')
        ax1.invert_yaxis()
    
    # Failed grasps
    if failures_u:
        ax2.hexbin(failures_v, failures_u, gridsize=20, cmap='Reds', mincnt=1)
        ax2.set_title('Failed Grasp Locations')
        ax2.set_xlabel('v (pixel column)')
        ax2.set_ylabel('u (pixel row)')
        ax2.invert_yaxis()
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'grasp_heatmap.png'), dpi=150)
    plt.close()
    print(f"  Generated: grasp_heatmap.png")


def plot_rotation_distribution(steps, output_dir):
    """Plot distribution of rotation choices"""
    rotation_stats = defaultdict(lambda: {'total': 0, 'successes': 0})
    
    for step in steps:
        rot_idx = step['predicted_grasp']['rotation_idx']
        rotation_stats[rot_idx]['total'] += 1
        if step['success']:
            rotation_stats[rot_idx]['successes'] += 1
    
    rotations = sorted(rotation_stats.keys())
    counts = [rotation_stats[r]['total'] for r in rotations]
    success_rates = [rotation_stats[r]['successes'] / rotation_stats[r]['total'] 
                     if rotation_stats[r]['total'] > 0 else 0 for r in rotations]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Usage frequency
    ax1.bar(rotations, counts, color='steelblue')
    ax1.set_ylabel('Frequency')
    ax1.set_title('Rotation Index Usage')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Success rate per rotation
    ax2.bar(rotations, success_rates, color='coral')
    ax2.set_xlabel('Rotation Index')
    ax2.set_ylabel('Success Rate')
    ax2.set_title('Success Rate by Rotation')
    ax2.set_ylim(0, 1.05)
    ax2.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'rotation_analysis.png'), dpi=150)
    plt.close()
    print(f"  Generated: rotation_analysis.png")


def generate_summary_stats(steps, output_dir):
    """Generate text summary of statistics"""
    total_steps = len(steps)
    successes = sum(1 for s in steps if s['success'])
    success_rate = successes / total_steps if total_steps > 0 else 0
    
    # Per-object stats
    object_stats = defaultdict(lambda: {'attempts': 0, 'successes': 0})
    for step in steps:
        obj_name = step.get('object_name', 'unknown')
        if obj_name != 'unknown':
            object_stats[obj_name]['attempts'] += 1
            if step['success']:
                object_stats[obj_name]['successes'] += 1
    
    # Exploration vs exploitation
    explore_count = sum(1 for s in steps if s['action_type'] == 'EXPLORE')
    exploit_count = sum(1 for s in steps if s['action_type'] == 'EXPLOIT')
    
    # Average confidence
    avg_confidence = np.mean([s['predicted_grasp']['confidence'] for s in steps])
    
    # Write summary
    summary_path = os.path.join(output_dir, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write("=" * 60 + "\n")
        f.write("TRAINING SUMMARY\n")
        f.write("=" * 60 + "\n\n")
        
        f.write(f"Total Steps: {total_steps}\n")
        f.write(f"Successes: {successes}\n")
        f.write(f"Failures: {total_steps - successes}\n")
        f.write(f"Overall Success Rate: {success_rate*100:.2f}%\n\n")
        
        f.write("-" * 60 + "\n")
        f.write("PER-OBJECT STATISTICS\n")
        f.write("-" * 60 + "\n")
        for obj_name in sorted(object_stats.keys()):
            stats = object_stats[obj_name]
            obj_rate = stats['successes'] / stats['attempts'] if stats['attempts'] > 0 else 0
            f.write(f"\n{obj_name}:\n")
            f.write(f"  Attempts: {stats['attempts']}\n")
            f.write(f"  Successes: {stats['successes']}\n")
            f.write(f"  Success Rate: {obj_rate*100:.2f}%\n")
        
        f.write("\n" + "-" * 60 + "\n")
        f.write("EXPLORATION STATISTICS\n")
        f.write("-" * 60 + "\n")
        f.write(f"Exploration actions: {explore_count} ({explore_count/total_steps*100:.1f}%)\n")
        f.write(f"Exploitation actions: {exploit_count} ({exploit_count/total_steps*100:.1f}%)\n\n")
        
        f.write("-" * 60 + "\n")
        f.write("NETWORK CONFIDENCE\n")
        f.write("-" * 60 + "\n")
        f.write(f"Average confidence: {avg_confidence:.4f}\n\n")
    
    print(f"  Generated: summary.txt")
    
    # Print to console
    print("\n" + "=" * 60)
    print("SUMMARY STATISTICS")
    print("=" * 60)
    print(f"Total Steps: {total_steps}")
    print(f"Overall Success Rate: {success_rate*100:.2f}%")
    print(f"Average Confidence: {avg_confidence:.4f}")


def main():
    parser = argparse.ArgumentParser(description='Analyze TossingBot training logs')
    parser.add_argument('--log', type=str, required=True,
                        help='Path to training log file (.jsonl)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory for plots (default: ./analysis_results/<timestamp>)')
    parser.add_argument('--window', type=int, default=50,
                        help='Moving average window size (default: 50)')
    
    args = parser.parse_args()
    
    # Validate input
    if not os.path.exists(args.log):
        print(f"Error: Log file not found: {args.log}")
        sys.exit(1)
    
    # Create output directory
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"analysis_results/{timestamp}"
    else:
        output_dir = args.output
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"\n{'='*60}")
    print(f"TossingBot Training Analysis")
    print(f"{'='*60}")
    print(f"Log file: {args.log}")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}\n")
    
    # Load log
    print("Loading log file...")
    entries = load_log(args.log)
    steps = extract_steps(entries)
    
    if len(steps) == 0:
        print("Error: No step data found in log file!")
        sys.exit(1)
    
    print(f"Loaded {len(steps)} training steps\n")
    
    # Generate plots
    print("Generating plots...")
    plot_success_rate(steps, output_dir, window=args.window)
    plot_per_object_success(steps, output_dir)
    plot_loss_curve(steps, output_dir, window=args.window)
    plot_epsilon_decay(steps, output_dir)
    plot_confidence_distribution(steps, output_dir)
    plot_grasp_heatmap(steps, output_dir)
    plot_rotation_distribution(steps, output_dir)
    
    # Generate summary
    print("\nGenerating summary statistics...")
    generate_summary_stats(steps, output_dir)
    
    print(f"\n{'='*60}")
    print(f"Analysis complete! Results saved to: {output_dir}")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
