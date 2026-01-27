#!/usr/bin/env python3.8
"""
Evaluate trained model on test objects.
Generates generalization metrics.
"""
import sys
import os
import json
import argparse
from collections import defaultdict

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot import config as cfg
from tossingbot.learning.session_manager import SessionManager


def compute_generalization_metrics(log_path, train_objects):
    """
    Calculate seen vs unseen performance metrics.
    
    Args:
        log_path: Path to training_log.jsonl
        train_objects: List of objects used in training
    
    Returns:
        dict with overall, seen, unseen, and per-object metrics
    """
    train_objects_set = set(train_objects) if train_objects else set()
    
    # Parse log
    per_object = defaultdict(lambda: {'attempts': 0, 'successes': 0})
    seen_success, seen_total = 0, 0
    unseen_success, unseen_total = 0, 0
    overall_success, overall_total = 0, 0
    
    with open(log_path, 'r') as f:
        for line in f:
            try:
                entry = json.loads(line.strip())
                
                # Skip episode markers
                if entry.get('event_type') == 'episode_end':
                    continue
                
                obj_name = entry.get('object_name')
                success = entry.get('success', False)
                
                if obj_name and obj_name != 'unknown':
                    # Per-object stats
                    per_object[obj_name]['attempts'] += 1
                    if success:
                        per_object[obj_name]['successes'] += 1
                    
                    # Overall
                    overall_total += 1
                    if success:
                        overall_success += 1
                    
                    # Seen vs unseen
                    if obj_name in train_objects_set:
                        seen_total += 1
                        if success: seen_success += 1
                    else:
                        unseen_total += 1
                        if success: unseen_success += 1
                        
            except (json.JSONDecodeError, KeyError):
                continue
    
    # Calculate rates
    overall_rate = overall_success / overall_total if overall_total > 0 else 0
    seen_rate = seen_success / seen_total if seen_total > 0 else 0
    unseen_rate = unseen_success / unseen_total if unseen_total > 0 else 0
    
    # Per-object rates
    for obj_name in per_object:
        stats = per_object[obj_name]
        stats['rate'] = stats['successes'] / stats['attempts'] if stats['attempts'] > 0 else 0
    
    return {
        'overall': {
            'rate': overall_rate,
            'successes': overall_success,
            'attempts': overall_total
        },
        'seen': {
            'rate': seen_rate,
            'successes': seen_success,
            'attempts': seen_total
        },
        'unseen': {
            'rate': unseen_rate,
            'successes': unseen_success,
            'attempts': unseen_total
        },
        'per_object': dict(per_object)
    }


def print_evaluation_report(session, metrics, train_objects):
    """Pretty-print evaluation results"""
    print(f"\n{'='*70}")
    print(f"EVALUATION REPORT: {session.name}")
    print(f"{'='*70}")
    
    test_objects = session.metadata.get('test_objects', [])
    
    if train_objects:
        print(f"\nTraining Objects ({len(train_objects)}): {', '.join(train_objects)}")
    if test_objects:
        print(f"Test Objects ({len(test_objects)}): {', '.join(test_objects)}")
    
    print(f"\n{'='*70}")
    print("OVERALL RESULTS")
    print(f"{'='*70}")
    print(f"Success Rate: {metrics['overall']['rate']*100:.1f}% "
          f"({metrics['overall']['successes']}/{metrics['overall']['attempts']})")
    
    if metrics['seen']['attempts'] > 0:
        print(f"\nSeen Objects (in training): {metrics['seen']['rate']*100:.1f}% "
              f"({metrics['seen']['successes']}/{metrics['seen']['attempts']})")
    
    if metrics['unseen']['attempts'] > 0:
        print(f"Unseen Objects (held out): {metrics['unseen']['rate']*100:.1f}% "
              f"({metrics['unseen']['successes']}/{metrics['unseen']['attempts']})")
        gap = (metrics['seen']['rate'] - metrics['unseen']['rate']) * 100
        print(f"\nGeneralization Gap: {gap:+.1f} percentage points")
    
    print(f"\n{'='*70}")
    print("PER-OBJECT BREAKDOWN")
    print(f"{'='*70}")
    
    for obj_name in sorted(metrics['per_object'].keys()):
        stats = metrics['per_object'][obj_name]
        seen_marker = "✓ [SEEN]  " if obj_name in train_objects else "✗ [UNSEEN]"
        print(f"{seen_marker} {obj_name:12s}: {stats['rate']*100:5.1f}% "
              f"({stats['successes']}/{stats['attempts']})")
    
    print(f"{'='*70}\n")


def evaluate_session(session_name, output=True):
    """
    Evaluate a training session and generate report.
    
    Args:
        session_name: Session ID
        output: Whether to print report (default: True)
    
    Returns:
        metrics dict
    """
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    session = session_manager.load_session(session_name, 'training')
    
    if session is None:
        print(f"Error: Session not found: {session_name}")
        return None
    
    # Get training objects from metadata
    train_objects = session.metadata.get('train_objects', cfg.DEFAULT_TRAIN_OBJECTS)
    
    # Load and analyze log
    log_path = session.get_log_path()
    if not os.path.exists(log_path):
        print(f"Error: Log file not found: {log_path}")
        return None
    
    metrics = compute_generalization_metrics(log_path, train_objects)
    
    # Print report
    if output:
        print_evaluation_report(session, metrics, train_objects)
    
    # Save metrics to session metadata
    session.metadata['evaluation_metrics'] = metrics
    session.save_metadata()
    
    return metrics


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate trained model generalization'
    )
    parser.add_argument('--session', required=True,
                        help='Session ID to evaluate')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress detailed output')
    
    args = parser.parse_args()
    
    metrics = evaluate_session(args.session, output=not args.quiet)
    
    if metrics is None:
        sys.exit(1)
    
    sys.exit(0)


if __name__ == '__main__':
    main()
