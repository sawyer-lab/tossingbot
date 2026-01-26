#!/usr/bin/env python3.8
"""
Visualize grasp attempts per object in object-relative coordinates.
Shows where the network chooses to grasp different object types.
"""
import sys
import os
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import defaultdict

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot import config as cfg
from tossingbot.scripts import object_dimensions
from tossingbot.scripts import coordinate_transforms


def load_training_log(log_path):
    """Load training log and extract grasp data"""
    grasps_by_object = defaultdict(list)
    
    with open(log_path, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line.strip())
                
                # Skip non-step events
                if entry.get('event_type') == 'episode_end':
                    continue
                
                # Extract data
                object_name = entry.get('object_name')
                if not object_name or object_name == 'unknown':
                    continue
                
                # Get grasp pixel coordinates
                predicted_grasp = entry.get('predicted_grasp', {})
                u = predicted_grasp.get('u')
                v = predicted_grasp.get('v')
                
                if u is None or v is None:
                    continue
                
                # Convert pixel to world coordinates
                world_xy = coordinate_transforms.pixel_to_world(
                    u, v,
                    cfg.ROI_X, cfg.ROI_Y,
                    cfg.IMG_H, cfg.IMG_W
                )
                
                # Get object pose
                object_poses = entry.get('object_poses', {})
                object_pose = object_poses.get(object_name)
                
                if not object_pose:
                    continue
                
                # Transform to object frame
                grasp_obj = coordinate_transforms.world_to_object_frame_2d(
                    world_xy, object_pose
                )
                
                # Store
                grasps_by_object[object_name].append({
                    'x': grasp_obj[0],
                    'y': grasp_obj[1],
                    'success': entry.get('success', False),
                    'confidence': predicted_grasp.get('confidence', 0.0),
                    'step': entry.get('step', 0)
                })
                
            except (json.JSONDecodeError, KeyError, TypeError) as e:
                print(f"Warning: Skipping malformed log entry: {e}")
                continue
    
    return grasps_by_object


def plot_object_grasp_distribution(object_name, grasps, output_path):
    """
    Plot grasp distribution for a single object type.
    
    Args:
        object_name: str, object type
        grasps: list of dicts with 'x', 'y', 'success'
        output_path: where to save plot
    """
    if len(grasps) == 0:
        print(f"No grasps for {object_name}")
        return False
    
    # Separate by success/failure
    success_x = [g['x'] for g in grasps if g['success']]
    success_y = [g['y'] for g in grasps if g['success']]
    failure_x = [g['x'] for g in grasps if not g['success']]
    failure_y = [g['y'] for g in grasps if not g['success']]
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Draw object outline
    rectangles = object_dimensions.get_object_outline(object_name)
    for (x_min, y_min, width, height) in rectangles:
        rect = patches.Rectangle(
            (x_min, y_min), width, height,
            linewidth=2, edgecolor='black', facecolor='lightgray', alpha=0.3
        )
        ax.add_patch(rect)
    
    # Plot successful grasp attempts (failures have object_name="unknown")
    if success_x:
        ax.scatter(success_x, success_y, c='green', s=50, alpha=0.6,
                  label=f'Successful Grasps ({len(success_x)})', marker='o')
    
    # Set bounds based on object dimensions
    bounds = object_dimensions.get_object_bounds(object_name)
    margin = 0.02  # 2cm margin
    ax.set_xlim(bounds[0] - margin, bounds[1] + margin)
    ax.set_ylim(bounds[2] - margin, bounds[3] + margin)
    
    # Labels and title
    ax.set_xlabel('X (meters, object frame)', fontsize=12)
    ax.set_ylabel('Y (meters, object frame)', fontsize=12)
    ax.set_title(f'{object_name} - Successful Grasp Locations (Top-Down View)\n'
                f'{len(success_x)} successful grasps shown (failures not tracked per-object)',
                fontsize=14, fontweight='bold')
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    
    # Add coordinate frame indicator
    ax.arrow(bounds[0] - margin*0.5, bounds[2] - margin*0.5,
            0.01, 0, head_width=0.005, head_length=0.003, fc='blue', ec='blue')
    ax.text(bounds[0] - margin*0.5 + 0.012, bounds[2] - margin*0.5, 'X',
           fontsize=10, color='blue', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"Saved: {output_path}")
    return True


def plot_all_objects_comparison(grasps_by_object, output_path):
    """
    Create a comparison plot showing all object types side-by-side.
    """
    object_names = sorted(grasps_by_object.keys())
    n_objects = len(object_names)
    
    if n_objects == 0:
        return False
    
    # Create subplots
    cols = min(3, n_objects)
    rows = (n_objects + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 6*rows))
    if rows == 1 and cols == 1:
        axes = [axes]
    else:
        axes = axes.flatten() if n_objects > 1 else [axes]
    
    for idx, object_name in enumerate(object_names):
        ax = axes[idx]
        grasps = grasps_by_object[object_name]
        
        # Only successful grasps (failures have object_name="unknown")
        success_x = [g['x'] for g in grasps if g['success']]
        success_y = [g['y'] for g in grasps if g['success']]
        
        # Draw object outline
        rectangles = object_dimensions.get_object_outline(object_name)
        for (x_min, y_min, width, height) in rectangles:
            rect = patches.Rectangle(
                (x_min, y_min), width, height,
                linewidth=1.5, edgecolor='black', facecolor='lightgray', alpha=0.3
            )
            ax.add_patch(rect)
        
        # Plot successful grasps
        if success_x:
            ax.scatter(success_x, success_y, c='green', s=30, alpha=0.5, marker='o')
        
        # Set bounds
        bounds = object_dimensions.get_object_bounds(object_name)
        margin = 0.015
        ax.set_xlim(bounds[0] - margin, bounds[1] + margin)
        ax.set_ylim(bounds[2] - margin, bounds[3] + margin)
        
        # Title
        ax.set_title(f'{object_name}\n{len(success_x)} successful grasps',
                    fontsize=10, fontweight='bold')
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)
    
    # Hide unused subplots
    for idx in range(n_objects, len(axes)):
        axes[idx].axis('off')
    
    plt.suptitle('Successful Grasp Locations - All Objects (failures not shown)', 
                fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"Saved comparison: {output_path}")
    return True


def analyze_per_object_grasps(log_path, output_dir):
    """
    Main analysis function.
    
    Args:
        log_path: path to training_log.jsonl
        output_dir: where to save visualizations
    """
    print(f"Loading grasp data from: {log_path}")
    grasps_by_object = load_training_log(log_path)
    
    if not grasps_by_object:
        print("No valid grasp data found in log!")
        return False
    
    # Create output directory
    per_object_dir = os.path.join(output_dir, 'per_object')
    os.makedirs(per_object_dir, exist_ok=True)
    
    print(f"\nGenerating per-object visualizations...")
    print(f"Objects found: {list(grasps_by_object.keys())}")
    
    # Plot each object individually
    for object_name, grasps in grasps_by_object.items():
        output_path = os.path.join(per_object_dir, f'{object_name}_distribution.png')
        plot_object_grasp_distribution(object_name, grasps, output_path)
    
    # Plot comparison
    comparison_path = os.path.join(output_dir, 'all_objects_comparison.png')
    plot_all_objects_comparison(grasps_by_object, comparison_path)
    
    # Print summary
    print(f"\n{'='*70}")
    print("SUCCESSFUL GRASP SUMMARY BY OBJECT")
    print("(Note: Failures not tracked per-object in logs)")
    print(f"{'='*70}")
    for object_name in sorted(grasps_by_object.keys()):
        grasps = grasps_by_object[object_name]
        successes = sum(1 for g in grasps if g['success'])
        print(f"{object_name:15s}: {successes:4d} successful grasps")
    print(f"{'='*70}\n")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Visualize per-object grasp distributions in object-relative coordinates'
    )
    parser.add_argument('--log', required=True, help='Path to training_log.jsonl')
    parser.add_argument('--output', required=True, help='Output directory')
    
    args = parser.parse_args()
    
    success = analyze_per_object_grasps(args.log, args.output)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
