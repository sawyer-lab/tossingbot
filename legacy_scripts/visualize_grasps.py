#!/usr/bin/env python3
"""
Grasp Visualization Script for TossingBot
Overlays predicted grasp points on heightmaps
Can be used standalone or imported by session_tools.py
"""
import argparse
import json
import os
import sys
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime


def load_log(log_path):
    """Load training log from JSONL file"""
    entries = []
    with open(log_path, 'r') as f:
        for line in f:
            if line.strip():
                entry = json.loads(line)
                if entry.get('event_type') != 'episode_end':
                    entries.append(entry)
    return entries


def create_grasp_visualization(u, v, angle_deg, success, confidence, obj_name="unknown", img_size=(80, 80)):
    """
    Create a visualization of a grasp point
    Args:
        u, v: pixel coordinates
        angle_deg: grasp orientation in degrees
        success: whether grasp succeeded
        confidence: network confidence score
        obj_name: name of target object
        img_size: size of heightmap (H, W)
    """
    # Create blank image (grayscale heightmap placeholder)
    # In real use, you would load the actual heightmap from disk
    img = np.ones((img_size[0], img_size[1], 3), dtype=np.uint8) * 128
    
    # Color based on success
    color = (0, 255, 0) if success else (0, 0, 255)  # Green for success, red for failure
    
    # Draw crosshair at grasp point
    cv2.line(img, (v-10, u), (v+10, u), color, 2)
    cv2.line(img, (v, u-10), (v, u+10), color, 2)
    cv2.circle(img, (v, u), 3, color, -1)
    
    # Draw orientation arrow
    rad = np.deg2rad(angle_deg)
    arrow_len = 30
    end_v = int(v + arrow_len * np.cos(rad))
    end_u = int(u + arrow_len * np.sin(rad))
    cv2.arrowedLine(img, (v, u), (end_v, end_u), color, 2, tipLength=0.3)
    
    # Add text annotations
    status = "SUCCESS" if success else "FAILED"
    cv2.putText(img, f"{status}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    cv2.putText(img, f"Obj: {obj_name}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.putText(img, f"Conf: {confidence:.3f}", (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.putText(img, f"Angle: {angle_deg:.0f}deg", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.putText(img, f"({u},{v})", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    
    return img


def create_summary_montage(steps, output_path, max_samples=50):
    """
    Create a montage of multiple grasp visualizations
    Args:
        steps: list of log entries
        output_path: where to save the montage
        max_samples: maximum number of samples to include
    """
    # Sample steps (get diverse examples)
    if len(steps) > max_samples:
        # Get mix of successes and failures
        successes = [s for s in steps if s['success']]
        failures = [s for s in steps if not s['success']]
        
        n_success = min(len(successes), max_samples // 2)
        n_failure = min(len(failures), max_samples // 2)
        
        sampled = (successes[:n_success] + failures[:n_failure])
        # Sort by step number
        sampled.sort(key=lambda x: x['step'])
    else:
        sampled = steps
    
    # Create individual visualizations
    imgs = []
    for step in sampled:
        grasp = step['predicted_grasp']
        img = create_grasp_visualization(
            u=grasp['u'],
            v=grasp['v'],
            angle_deg=grasp['angle_deg'],
            success=step['success'],
            confidence=grasp['confidence'],
            obj_name=step.get('object_name', 'unknown')
        )
        imgs.append(img)
    
    if len(imgs) == 0:
        print("No images to create montage")
        return
    
    # Arrange in grid
    n_cols = 10
    n_rows = (len(imgs) + n_cols - 1) // n_cols
    
    # Pad with blank images if needed
    while len(imgs) < n_rows * n_cols:
        imgs.append(np.zeros_like(imgs[0]))
    
    # Create montage
    rows = []
    for i in range(n_rows):
        row_imgs = imgs[i*n_cols:(i+1)*n_cols]
        rows.append(np.hstack(row_imgs))
    
    montage = np.vstack(rows)
    
    # Save
    cv2.imwrite(output_path, montage)
    print(f"  Saved montage: {output_path}")


def create_best_worst_comparison(steps, output_dir):
    """Create side-by-side comparison of best and worst grasps"""
    # Sort by confidence
    sorted_steps = sorted(steps, key=lambda x: x['predicted_grasp']['confidence'])
    
    # Get top and bottom examples
    n_examples = 5
    worst = sorted_steps[:n_examples]
    best = sorted_steps[-n_examples:]
    
    # Create visualizations
    worst_imgs = []
    for step in worst:
        grasp = step['predicted_grasp']
        img = create_grasp_visualization(
            u=grasp['u'],
            v=grasp['v'],
            angle_deg=grasp['angle_deg'],
            success=step['success'],
            confidence=grasp['confidence'],
            obj_name=step.get('object_name', 'unknown')
        )
        worst_imgs.append(img)
    
    best_imgs = []
    for step in best:
        grasp = step['predicted_grasp']
        img = create_grasp_visualization(
            u=grasp['u'],
            v=grasp['v'],
            angle_deg=grasp['angle_deg'],
            success=step['success'],
            confidence=grasp['confidence'],
            obj_name=step.get('object_name', 'unknown')
        )
        best_imgs.append(img)
    
    # Combine
    worst_row = np.hstack(worst_imgs)
    best_row = np.hstack(best_imgs)
    
    # Add labels
    label_height = 30
    label_worst = np.zeros((label_height, worst_row.shape[1], 3), dtype=np.uint8)
    label_best = np.zeros((label_height, best_row.shape[1], 3), dtype=np.uint8)
    
    cv2.putText(label_worst, "LOWEST CONFIDENCE", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(label_best, "HIGHEST CONFIDENCE", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Stack with labels
    comparison = np.vstack([label_worst, worst_row, label_best, best_row])
    
    output_path = os.path.join(output_dir, 'confidence_comparison.png')
    cv2.imwrite(output_path, comparison)
    print(f"  Saved comparison: {output_path}")


def create_per_object_examples(steps, output_dir):
    """Create example visualizations for each object type"""
    # Group by object
    by_object = {}
    for step in steps:
        obj_name = step.get('object_name', 'unknown')
        if obj_name != 'unknown':
            if obj_name not in by_object:
                by_object[obj_name] = []
            by_object[obj_name].append(step)
    
    # Create visualization for each object
    for obj_name, obj_steps in by_object.items():
        # Get some successes and failures
        successes = [s for s in obj_steps if s['success']]
        failures = [s for s in obj_steps if not s['success']]
        
        examples = successes[:5] + failures[:5]
        
        imgs = []
        for step in examples:
            grasp = step['predicted_grasp']
            img = create_grasp_visualization(
                u=grasp['u'],
                v=grasp['v'],
                angle_deg=grasp['angle_deg'],
                success=step['success'],
                confidence=grasp['confidence'],
                obj_name=obj_name
            )
            imgs.append(img)
        
        if imgs:
            combined = np.hstack(imgs)
            output_path = os.path.join(output_dir, f'object_{obj_name}_examples.png')
            cv2.imwrite(output_path, combined)
            print(f"  Saved examples for {obj_name}: {output_path}")


def visualize_session_from_path(log_path, output_dir, max_samples=100):
    """
    Visualize grasps from training log.
    This is the main function that can be called by session_tools.py
    
    Args:
        log_path: Path to training log file
        output_dir: Directory to save visualizations
        max_samples: Maximum number of samples for montage
    
    Returns:
        bool: True if successful
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"\n{'='*60}")
    print(f"TossingBot Grasp Visualization")
    print(f"{'='*60}")
    print(f"Log file: {log_path}")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}\n")
    
    # Load log
    print("Loading log file...")
    steps = load_log(log_path)
    
    if len(steps) == 0:
        print("Error: No step data found in log file!")
        return False
    
    print(f"Loaded {len(steps)} training steps\n")
    
    # Generate visualizations
    print("Creating visualizations...")
    
    # Overall montage
    montage_path = os.path.join(output_dir, 'grasp_montage.png')
    create_summary_montage(steps, montage_path, max_samples=max_samples)
    
    # Best vs worst
    create_best_worst_comparison(steps, output_dir)
    
    # Per-object examples
    create_per_object_examples(steps, output_dir)
    
    print(f"\n{'='*60}")
    print(f"Visualization complete! Results saved to: {output_dir}")
    print(f"{'='*60}\n")
    print("Note: These visualizations use placeholder heightmaps.")
    print("To overlay on actual heightmaps, you would need to save")
    print("heightmap images during training and load them here.")
    
    return True


def visualize_session(session, samples=100):
    """
    Visualize grasps for a training session object.
    Convenience wrapper for use with SessionManager.
    
    Args:
        session: Session object from SessionManager
        samples: Maximum number of samples for montage
    
    Returns:
        bool: True if successful
    """
    log_path = session.get_log_path()
    output_dir = os.path.join(session.analysis_dir, "visualizations")
    
    if not os.path.exists(log_path):
        print(f"Error: Log file not found at {log_path}")
        print("Train the session first to generate logs.")
        return False
    
    print(f"Visualizing session: {session.name}")
    return visualize_session_from_path(log_path, output_dir, samples)


def main():
    parser = argparse.ArgumentParser(
        description='Visualize TossingBot grasps from training logs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Visualize from log file path
  %(prog)s --log logs/training_log.jsonl
  
  # Custom output directory
  %(prog)s --log logs/training_log.jsonl --output my_viz
  
  # Control montage size
  %(prog)s --log logs/training_log.jsonl --samples 200

Note:
  For session-based visualization, use session_tools.py instead:
    python session_tools.py visualize --session <session_id>
        '''
    )
    parser.add_argument('--log', type=str, required=True,
                        help='Path to training log file (.jsonl)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory for visualizations (default: ./visualization_results/<timestamp>)')
    parser.add_argument('--samples', type=int, default=100,
                        help='Maximum number of samples for montage (default: 100)')
    
    args = parser.parse_args()
    
    # Validate input
    if not os.path.exists(args.log):
        print(f"Error: Log file not found: {args.log}")
        sys.exit(1)
    
    # Create output directory
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"visualization_results/{timestamp}"
    else:
        output_dir = args.output
    
    # Run visualization
    success = visualize_session_from_path(args.log, output_dir, args.samples)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
