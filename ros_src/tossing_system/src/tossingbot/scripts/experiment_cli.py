#!/usr/bin/env python3.8
"""
Unified CLI for TossingBot Experiment Management
Manages experiment lifecycle: create, run, analyze, compare
"""
import sys
import os
import argparse
import subprocess

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tossingbot import config as cfg
from tossingbot.learning.experiment_manager import ExperimentManager
from tossingbot.learning.experiment_session import ExperimentSession


def cmd_create(args):
    """Create new experiment from config file"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    try:
        experiment = manager.create_experiment_from_file(args.config)
        print(f"\nExperiment created successfully: {experiment.experiment_id}")
        print(f"  Config: {experiment.config_path}")
        print(f"\nNext steps:")
        print(f"  1. Run training: python {sys.argv[0]} run --experiment {experiment.experiment_id} --phase train")
        print(f"  2. Run evals: python {sys.argv[0]} run --experiment {experiment.experiment_id} --phase eval --eval-name all")
        print(f"  3. Analyze: python {sys.argv[0]} analyze --experiment {experiment.experiment_id}")
    except Exception as e:
        print(f"Error creating experiment: {e}")
        sys.exit(1)


def cmd_list(args):
    """List all experiments"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    experiments = manager.list_experiments(status=args.status)

    print(f"\n{'=' * 70}")
    print("EXPERIMENTS")
    print(f"{'=' * 70}")

    if not experiments:
        print("No experiments found.")
    else:
        for exp in experiments:
            try:
                config = exp.load_config()
                name = config.get('name', exp.experiment_id)
                desc = config.get('description', 'No description')
                status = exp.get_status()
                created = exp.metadata.get('created_at', 'unknown')[:10]
                updated = exp.metadata.get('last_updated', 'unknown')[:10]

                train_completed = exp.is_training_completed()
                eval_phases = exp.list_eval_phases()
                eval_completed = len([e for e in eval_phases if exp.is_eval_completed(e)])

                print(f"\n{name} [{status}]")
                print(f"  {desc}")
                print(f"  Created: {created}, Updated: {updated}")
                print(f"  Training: {'✓' if train_completed else '○'}, " +
                      f"Evals: {eval_completed}/{len(eval_phases)} completed")
            except Exception as e:
                print(f"\n{exp.experiment_id} [ERROR: {e}]")

    print(f"\n{'=' * 70}\n")


def cmd_info(args):
    """Show detailed experiment information"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    experiment = manager.load_experiment(args.experiment)
    if experiment is None:
        print(f"Error: Experiment not found: {args.experiment}")
        sys.exit(1)

    manager.print_experiment_info(experiment, verbose=args.verbose)


def cmd_list_evals(args):
    """List evaluation phases for an experiment"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    experiment = manager.load_experiment(args.experiment)
    if experiment is None:
        print(f"Error: Experiment not found: {args.experiment}")
        sys.exit(1)

    config = experiment.load_config()
    eval_phases = experiment.list_eval_phases()

    print(f"\n{'=' * 70}")
    print(f"EVALUATION PHASES: {experiment.experiment_id}")
    print(f"{'=' * 70}\n")

    if not eval_phases:
        print("No evaluation phases defined.")
    else:
        evals_config = config.get('evals', {})
        for eval_name in eval_phases:
            is_completed = experiment.is_eval_completed(eval_name)
            status_mark = "✓" if is_completed else "○"

            eval_cfg = evals_config.get(eval_name, {})
            desc = eval_cfg.get('description', 'No description')
            objects = eval_cfg.get('objects', [])
            episodes = eval_cfg.get('episodes', 0)
            checkpoint = eval_cfg.get('checkpoint', 'best')

            print(f"{status_mark} {eval_name}")
            print(f"    {desc}")
            print(f"    Objects: {', '.join(objects)}")
            print(f"    Episodes: {episodes}, Checkpoint: {checkpoint}")
            print()

    print(f"{'=' * 70}\n")


def cmd_run(args):
    """Run experiment phases"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    experiment = manager.load_experiment(args.experiment)
    if experiment is None:
        print(f"Error: Experiment not found: {args.experiment}")
        sys.exit(1)

    config = experiment.load_config()
    script_dir = os.path.dirname(os.path.dirname(__file__))
    auto_grasp_path = os.path.join(script_dir, 'scripts', 'auto_grasp.py')

    # Determine what to run
    if args.phase == 'train':
        # Run training phase
        print(f"\n{'=' * 70}")
        print(f"RUNNING TRAINING PHASE: {experiment.experiment_id}")
        print(f"{'=' * 70}\n")

        train_objects = config['train']['objects']
        print(f"Training objects: {', '.join(train_objects)}")
        print(f"Target steps: {config['train'].get('steps', 'unlimited')}")
        print()

        # Update status
        experiment.set_status('training')

        # Build command
        cmd = [
            'python3.8', auto_grasp_path,
            '--experiment', experiment.experiment_id,
            '--phase', 'train'
        ]

        print(f"Command: {' '.join(cmd)}\n")

        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            print("\n\nTraining interrupted by user.")
        except Exception as e:
            print(f"Error running training: {e}")
            sys.exit(1)

    elif args.phase == 'eval':
        # Run evaluation phase(s)
        eval_phases = config.get('evals', {})

        if not eval_phases:
            print("Error: No evaluation phases defined in config")
            sys.exit(1)

        # Determine which eval phases to run
        if args.eval_name == 'all':
            eval_names_to_run = list(eval_phases.keys())
        elif args.eval_name:
            if args.eval_name not in eval_phases:
                print(f"Error: Eval phase '{args.eval_name}' not defined")
                print(f"Available: {list(eval_phases.keys())}")
                sys.exit(1)
            eval_names_to_run = [args.eval_name]
        else:
            print("Error: --eval-name required for eval phase")
            print(f"Available: {list(eval_phases.keys())} or 'all'")
            sys.exit(1)

        # Update status
        experiment.set_status('evaluating')

        # Run each eval phase
        for eval_name in eval_names_to_run:
            print(f"\n{'=' * 70}")
            print(f"RUNNING EVAL PHASE: {eval_name}")
            print(f"{'=' * 70}\n")

            eval_cfg = eval_phases[eval_name]
            print(f"Description: {eval_cfg.get('description', 'N/A')}")
            print(f"Objects: {', '.join(eval_cfg['objects'])}")
            print(f"Episodes: {eval_cfg['episodes']}")
            print(f"Checkpoint: {eval_cfg.get('checkpoint', 'best')}")
            print()

            # Build command
            cmd = [
                'python3.8', auto_grasp_path,
                '--experiment', experiment.experiment_id,
                '--phase', 'eval',
                '--eval-name', eval_name
            ]

            print(f"Command: {' '.join(cmd)}\n")

            try:
                subprocess.run(cmd, check=True)
                print(f"\n✓ Eval phase '{eval_name}' completed\n")
            except KeyboardInterrupt:
                print(f"\n\nEval phase '{eval_name}' interrupted by user.")
                break
            except subprocess.CalledProcessError as e:
                print(f"Error running eval phase '{eval_name}': {e}")
                sys.exit(1)

    elif args.phase == 'all':
        # Run full experiment: train + all evals
        print(f"\n{'=' * 70}")
        print(f"RUNNING FULL EXPERIMENT: {experiment.experiment_id}")
        print(f"{'=' * 70}\n")
        print("This will run training, then all evaluation phases.")
        print()

        # Run training
        train_args = argparse.Namespace(
            experiment=args.experiment,
            phase='train'
        )
        cmd_run(train_args)

        # Run all evals
        eval_args = argparse.Namespace(
            experiment=args.experiment,
            phase='eval',
            eval_name='all'
        )
        cmd_run(eval_args)

        # Mark as completed
        experiment.set_status('completed')


def cmd_analyze(args):
    """Analyze experiment results"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    experiment = manager.load_experiment(args.experiment)
    if experiment is None:
        print(f"Error: Experiment not found: {args.experiment}")
        sys.exit(1)

    print(f"\n{'=' * 70}")
    print(f"ANALYZING EXPERIMENT: {experiment.experiment_id}")
    print(f"{'=' * 70}\n")

    # Import analysis module
    try:
        from tossingbot.scripts import analyze_experiment
    except ImportError:
        print("Error: analyze_experiment module not found")
        print("This feature is not yet implemented.")
        sys.exit(1)

    # Determine which eval phases to analyze
    eval_names = args.eval_names if args.eval_names else None

    # Run analysis
    success = analyze_experiment.analyze_experiment(experiment, eval_names)

    if success:
        print(f"\n{'=' * 70}")
        print(f"Analysis complete!")
        print(f"Results saved to: {experiment.analysis_dir}")
        print(f"{'=' * 70}\n")
        sys.exit(0)
    else:
        print("\nAnalysis failed.")
        sys.exit(1)


def cmd_compare(args):
    """Compare multiple experiments"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    manager.compare_experiments(args.experiments, eval_name=args.eval_name)


def cmd_delete(args):
    """Delete experiment"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    manager.delete_experiment(args.experiment, force=args.force)


def cmd_archive(args):
    """Archive experiment"""
    manager = ExperimentManager(cfg.SESSION_BASE_DIR + "/experiments")

    manager.archive_experiment(args.experiment)


def main():
    parser = argparse.ArgumentParser(
        description='TossingBot Experiment Management CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Create experiment from config
  %(prog)s create --config experiments/exp_baseline.yaml

  # Run full experiment (train + all evals)
  %(prog)s run --experiment exp_baseline --phase all

  # Run specific phases
  %(prog)s run --experiment exp_baseline --phase train
  %(prog)s run --experiment exp_baseline --phase eval --eval-name seen
  %(prog)s run --experiment exp_baseline --phase eval --eval-name all

  # List experiments
  %(prog)s list
  %(prog)s list --status completed

  # Show experiment info
  %(prog)s info --experiment exp_baseline
  %(prog)s list-evals --experiment exp_baseline

  # Analyze results
  %(prog)s analyze --experiment exp_baseline
  %(prog)s analyze --experiment exp_baseline --eval-names seen unseen

  # Compare experiments
  %(prog)s compare --experiments exp_baseline exp_variant

  # Archive/delete
  %(prog)s archive --experiment old_exp
  %(prog)s delete --experiment bad_exp --force
        '''
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to execute')

    # Create command
    parser_create = subparsers.add_parser('create', help='Create new experiment')
    parser_create.add_argument('--config', required=True, help='Path to experiment config YAML')

    # List command
    parser_list = subparsers.add_parser('list', help='List experiments')
    parser_list.add_argument('--status', help='Filter by status')

    # Info command
    parser_info = subparsers.add_parser('info', help='Show experiment details')
    parser_info.add_argument('--experiment', required=True, help='Experiment ID')
    parser_info.add_argument('--verbose', '-v', action='store_true', help='Show detailed info')

    # List-evals command
    parser_list_evals = subparsers.add_parser('list-evals', help='List evaluation phases')
    parser_list_evals.add_argument('--experiment', required=True, help='Experiment ID')

    # Run command
    parser_run = subparsers.add_parser('run', help='Run experiment phases')
    parser_run.add_argument('--experiment', required=True, help='Experiment ID')
    parser_run.add_argument('--phase', choices=['train', 'eval', 'all'], default='train',
                           help='Phase to run (train, eval, or all)')
    parser_run.add_argument('--eval-name', help='Eval phase name (for --phase eval), or "all"')

    # Analyze command
    parser_analyze = subparsers.add_parser('analyze', help='Analyze experiment')
    parser_analyze.add_argument('--experiment', required=True, help='Experiment ID')
    parser_analyze.add_argument('--eval-names', nargs='+', help='Specific eval phases to analyze')

    # Compare command
    parser_compare = subparsers.add_parser('compare', help='Compare experiments')
    parser_compare.add_argument('--experiments', nargs='+', required=True, help='Experiment IDs')
    parser_compare.add_argument('--eval-name', help='Specific eval phase to compare')

    # Delete command
    parser_delete = subparsers.add_parser('delete', help='Delete experiment')
    parser_delete.add_argument('--experiment', required=True, help='Experiment ID')
    parser_delete.add_argument('--force', action='store_true', help='Skip confirmation')

    # Archive command
    parser_archive = subparsers.add_parser('archive', help='Archive experiment')
    parser_archive.add_argument('--experiment', required=True, help='Experiment ID')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Route to command handler
    command_map = {
        'create': cmd_create,
        'list': cmd_list,
        'info': cmd_info,
        'list-evals': cmd_list_evals,
        'run': cmd_run,
        'analyze': cmd_analyze,
        'compare': cmd_compare,
        'delete': cmd_delete,
        'archive': cmd_archive,
    }

    command_map[args.command](args)


if __name__ == '__main__':
    main()
