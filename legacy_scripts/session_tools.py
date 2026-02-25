#!/usr/bin/env python3.8
"""
Session Tools for TossingBot
Unified interface for session management and analysis
"""
import argparse
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot import config as cfg
from tossingbot.learning.session_manager import SessionManager, Session
from tossingbot.scripts import analyze_training
from tossingbot.scripts import visualize_grasps


def cmd_list(args):
    """List all sessions"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    # Determine which modes to list
    modes = []
    if args.mode in ['all', 'training']:
        modes.append('training')
    if args.mode in ['all', 'demo']:
        modes.append('demo')
    
    for mode in modes:
        sessions = session_manager.list_sessions(mode)
        
        print(f"\n{'='*70}")
        print(f"{mode.upper()} SESSIONS")
        print(f"{'='*70}")
        
        if not sessions:
            print(f"  No {mode} sessions found.")
        else:
            for session in sessions:
                created = session.metadata.get('created_at', 'unknown')[:19]
                last_run = session.metadata.get('last_run', 'unknown')[:19]
                steps = session.metadata.get('total_steps', 0)
                success = session.metadata.get('success_rate', 0.0) * 100
                status = session.metadata.get('status', 'active')
                
                status_mark = "[ARCHIVED]" if status == 'archived' else ""
                
                print(f"\n  {session.session_id} {status_mark}")
                print(f"    Name: {session.name}")
                print(f"    Created: {created}")
                print(f"    Last run: {last_run}")
                if mode == 'training':
                    print(f"    Steps: {steps}, Success rate: {success:.1f}%")
                    best = session.metadata.get('best_checkpoint')
                    if best:
                        print(f"    Best checkpoint: step {best['step']} ({best['success_rate']*100:.1f}% success)")
        
        print()


def cmd_info(args):
    """Show detailed session information"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    # Try loading as training session first
    session = session_manager.load_session(args.session, 'training')
    if session is None:
        # Try demo session
        session = session_manager.load_session(args.session, 'demo')
    
    if session is None:
        print(f"Error: Session not found: {args.session}")
        sys.exit(1)
    
    metadata = session.metadata
    
    print(f"\n{'='*70}")
    print(f"SESSION INFORMATION")
    print(f"{'='*70}")
    print(f"Session ID: {session.session_id}")
    print(f"Name: {session.name}")
    print(f"Mode: {session.mode}")
    print(f"Status: {metadata.get('status', 'active')}")
    print(f"Directory: {session.session_dir}")
    print()
    print(f"TIMESTAMPS:")
    print(f"  Created: {metadata.get('created_at', 'unknown')}")
    print(f"  Last run: {metadata.get('last_run', 'unknown')}")
    print(f"  Last updated: {metadata.get('last_updated', 'unknown')}")
    print()
    
    if session.mode == 'training':
        print(f"TRAINING STATISTICS:")
        print(f"  Total steps: {metadata.get('total_steps', 0)}")
        print(f"  Total episodes: {metadata.get('total_episodes', 0)}")
        print(f"  Success rate: {metadata.get('success_rate', 0.0)*100:.2f}%")
        print(f"  Buffer size: {metadata.get('buffer_size', 0)} experiences")
        print()
        
        print(f"CHECKPOINTS:")
        checkpoints = metadata.get('checkpoints', [])
        if checkpoints:
            for ckpt in checkpoints[-5:]:  # Show last 5
                print(f"  - {ckpt}")
            if len(checkpoints) > 5:
                print(f"  ... and {len(checkpoints) - 5} more")
        else:
            print(f"  No checkpoints saved yet")
        print()
        
        best = metadata.get('best_checkpoint')
        if best:
            print(f"BEST CHECKPOINT:")
            print(f"  File: {best['file']}")
            print(f"  Step: {best['step']}")
            print(f"  Success rate: {best['success_rate']*100:.2f}%")
            print(f"  Timestamp: {best.get('timestamp', 'unknown')}")
            print()
        
        hyperparams = metadata.get('hyperparameters', {})
        if hyperparams:
            print(f"HYPERPARAMETERS:")
            for key, value in hyperparams.items():
                print(f"  {key}: {value}")
            print()
    
    elif session.mode == 'demo':
        print(f"DEMO INFORMATION:")
        print(f"  Source training session: {metadata.get('source_training_session', 'unknown')}")
        print(f"  Source checkpoint: {metadata.get('source_checkpoint', 'unknown')}")
        print(f"  Total steps: {metadata.get('total_steps', 0)}")
        print()
    
    print(f"{'='*70}\n")


def cmd_analyze(args):
    """Analyze a training session"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    # Get session
    if args.session:
        session = session_manager.load_session(args.session, 'training')
        if session is None:
            print(f"Error: Training session not found: {args.session}")
            sys.exit(1)
    else:
        # Interactive selection
        print("Select session to analyze:")
        session = session_manager.select_session_interactive('training')
    
    # Run analysis
    success = analyze_training.analyze_session(session, window=args.window)
    sys.exit(0 if success else 1)


def cmd_visualize(args):
    """Visualize grasps for a session"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    # Get session
    if args.session:
        session = session_manager.load_session(args.session, 'training')
        if session is None:
            print(f"Error: Training session not found: {args.session}")
            sys.exit(1)
    else:
        # Interactive selection
        print("Select session to visualize:")
        session = session_manager.select_session_interactive('training')
    
    # Run visualization
    success = visualize_grasps.visualize_session(session, samples=args.samples)
    sys.exit(0 if success else 1)


def cmd_compare(args):
    """Compare multiple sessions"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    if len(args.sessions) < 2:
        print("Error: Need at least 2 sessions to compare")
        sys.exit(1)
    
    # Load sessions
    sessions = []
    for session_id in args.sessions:
        session = session_manager.load_session(session_id, 'training')
        if session is None:
            print(f"Error: Session not found: {session_id}")
            sys.exit(1)
        sessions.append(session)
    
    # Generate comparison report
    print(f"\n{'='*70}")
    print(f"SESSION COMPARISON")
    print(f"{'='*70}\n")
    
    # Header
    header = "Metric".ljust(30)
    for session in sessions:
        header += session.name[:15].ljust(18)
    print(header)
    print("-" * 70)
    
    # Compare metrics
    metrics = [
        ('Total Steps', 'total_steps', lambda x: f"{x}"),
        ('Total Episodes', 'total_episodes', lambda x: f"{x}"),
        ('Success Rate', 'success_rate', lambda x: f"{x*100:.1f}%"),
        ('Buffer Size', 'buffer_size', lambda x: f"{x}"),
    ]
    
    for label, key, formatter in metrics:
        row = label.ljust(30)
        values = []
        for session in sessions:
            value = session.metadata.get(key, 0)
            row += formatter(value).ljust(18)
            values.append(value)
        
        # Add winner indicator for comparable metrics
        if key in ['success_rate'] and all(v > 0 for v in values):
            winner_idx = values.index(max(values))
            row += f"  <- {sessions[winner_idx].name}"
        
        print(row)
    
    # Best checkpoints
    print()
    print("Best Checkpoints:".ljust(30))
    for session in sessions:
        best = session.metadata.get('best_checkpoint')
        if best:
            info = f"step {best['step']} ({best['success_rate']*100:.1f}%)"
        else:
            info = "none"
        print(f"  {session.name}: {info}")
    
    # Hyperparameters comparison
    print(f"\n{'='*70}")
    print("HYPERPARAMETER DIFFERENCES")
    print(f"{'='*70}\n")
    
    # Collect all hyperparameters
    all_params = set()
    for session in sessions:
        all_params.update(session.metadata.get('hyperparameters', {}).keys())
    
    if all_params:
        for param in sorted(all_params):
            row = param.ljust(30)
            values = []
            for session in sessions:
                value = session.metadata.get('hyperparameters', {}).get(param, 'N/A')
                row += str(value).ljust(18)
                values.append(value)
            
            # Only print if values differ
            if len(set(str(v) for v in values)) > 1:
                print(row)
    else:
        print("No hyperparameter data available")
    
    print(f"\n{'='*70}\n")


def cmd_archive(args):
    """Archive a session"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    session_manager.archive_session(args.session, 'training')


def cmd_delete(args):
    """Delete a session"""
    session_manager = SessionManager(cfg.SESSION_BASE_DIR)
    
    if not args.force:
        response = input(f"Are you sure you want to delete '{args.session}'? (yes/no): ")
        if response.lower() not in ['yes', 'y']:
            print("Cancelled.")
            return
    
    session_manager.delete_session(args.session, 'training')


def main():
    parser = argparse.ArgumentParser(
        description='TossingBot Session Management and Analysis Tools',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # List all sessions
  %(prog)s list
  
  # Show session details
  %(prog)s info --session session_baseline_v1
  
  # Analyze training session (interactive if no session specified)
  %(prog)s analyze --session session_baseline_v1
  %(prog)s analyze  # Interactive selection
  
  # Visualize grasps
  %(prog)s visualize --session session_baseline_v1
  
  # Compare two sessions
  %(prog)s compare --sessions session_baseline_v1 session_experiment_v2
  
  # Archive old session
  %(prog)s archive --session session_old_v1
  
  # Delete session (with confirmation)
  %(prog)s delete --session session_old_v1

Workflow:
  1. Train: python auto_grasp.py --mode training
  2. Analyze: python session_tools.py analyze --session <name>
  3. Compare: python session_tools.py compare --sessions <name1> <name2>
  4. Demo: python auto_grasp.py --mode demo --session <name>
        '''
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # List command
    parser_list = subparsers.add_parser('list', help='List all sessions')
    parser_list.add_argument('--mode', choices=['training', 'demo', 'all'], 
                            default='all', help='Session mode to list (default: all)')
    
    # Info command
    parser_info = subparsers.add_parser('info', help='Show session information')
    parser_info.add_argument('--session', required=True, help='Session ID')
    
    # Analyze command
    parser_analyze = subparsers.add_parser('analyze', help='Analyze training session')
    parser_analyze.add_argument('--session', help='Session ID (interactive if not provided)')
    parser_analyze.add_argument('--window', type=int, default=50, 
                               help='Moving average window size (default: 50)')
    
    # Visualize command
    parser_visualize = subparsers.add_parser('visualize', help='Visualize grasps')
    parser_visualize.add_argument('--session', help='Session ID (interactive if not provided)')
    parser_visualize.add_argument('--samples', type=int, default=100,
                                 help='Number of samples for montage (default: 100)')
    
    # Compare command
    parser_compare = subparsers.add_parser('compare', help='Compare sessions')
    parser_compare.add_argument('--sessions', nargs='+', required=True,
                               help='Session IDs to compare (space-separated)')
    
    # Archive command
    parser_archive = subparsers.add_parser('archive', help='Archive a session')
    parser_archive.add_argument('--session', required=True, help='Session ID')
    
    # Delete command
    parser_delete = subparsers.add_parser('delete', help='Delete a session')
    parser_delete.add_argument('--session', required=True, help='Session ID')
    parser_delete.add_argument('--force', action='store_true',
                              help='Skip confirmation prompt')
    
    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        sys.exit(1)
    
    # Route to command handler
    command_map = {
        'list': cmd_list,
        'info': cmd_info,
        'analyze': cmd_analyze,
        'visualize': cmd_visualize,
        'compare': cmd_compare,
        'archive': cmd_archive,
        'delete': cmd_delete,
    }
    
    command_map[args.command](args)


if __name__ == '__main__':
    main()
