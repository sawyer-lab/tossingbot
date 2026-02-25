"""
Experiment Manager for TossingBot
Manages experiment lifecycle: creation, execution, analysis
"""
import os
import sys
import yaml
import datetime
from typing import List, Optional, Dict, Any
from pathlib import Path

from tossingbot.learning.experiment_session import ExperimentSession


class ExperimentManager:
    """Manages experiment lifecycle and operations"""

    def __init__(self, base_dir: str = "sessions/experiments"):
        """
        Initialize experiment manager.

        Args:
            base_dir: Base directory for all experiments
        """
        self.base_dir = base_dir
        os.makedirs(base_dir, exist_ok=True)

    def create_experiment(self, config: Dict[str, Any]) -> ExperimentSession:
        """
        Create new experiment from configuration.

        Args:
            config: Experiment configuration dictionary

        Returns:
            ExperimentSession object

        Raises:
            ValueError: If config is invalid or experiment already exists
        """
        # Validate config
        self._validate_config(config)

        # Get experiment name/ID
        experiment_id = config.get('name')
        if not experiment_id:
            # Generate ID from timestamp if no name provided
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            experiment_id = f"exp_{timestamp}"
            config['name'] = experiment_id

        # Check if experiment already exists
        experiment_path = os.path.join(self.base_dir, experiment_id)
        if os.path.exists(experiment_path):
            raise ValueError(f"Experiment already exists: {experiment_id}")

        # Create experiment session
        experiment = ExperimentSession(experiment_id, self.base_dir)

        # Add creation timestamp to config
        if 'created' not in config:
            config['created'] = datetime.datetime.now().isoformat()

        # Save config
        experiment.save_config(config)

        # Initialize metadata
        experiment.metadata['experiment_name'] = config.get('name')
        experiment.metadata['description'] = config.get('description', '')
        experiment.set_status('created')

        print(f"Created experiment: {experiment_id}")
        print(f"  Location: {experiment.experiment_dir}")
        print(f"  Training objects: {config.get('train', {}).get('objects', [])}")
        print(f"  Evaluation phases: {list(config.get('evals', {}).keys())}")

        return experiment

    def create_experiment_from_file(self, config_path: str) -> ExperimentSession:
        """
        Create experiment from YAML config file.

        Args:
            config_path: Path to YAML config file

        Returns:
            ExperimentSession object
        """
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            raise ValueError(f"Could not load config file: {e}")

        return self.create_experiment(config)

    def load_experiment(self, experiment_id: str) -> Optional[ExperimentSession]:
        """
        Load existing experiment.

        Args:
            experiment_id: Experiment identifier

        Returns:
            ExperimentSession object or None if not found
        """
        experiment_path = os.path.join(self.base_dir, experiment_id)

        if not os.path.exists(experiment_path):
            print(f"Experiment not found: {experiment_id}")
            return None

        try:
            experiment = ExperimentSession(experiment_id, self.base_dir)
            return experiment
        except Exception as e:
            print(f"Error loading experiment: {e}")
            return None

    def list_experiments(self, status: str = None) -> List[ExperimentSession]:
        """
        List all experiments.

        Args:
            status: Filter by status (created, training, evaluating, completed, archived)

        Returns:
            List of ExperimentSession objects
        """
        experiments = []

        if not os.path.exists(self.base_dir):
            return experiments

        for experiment_id in os.listdir(self.base_dir):
            experiment_path = os.path.join(self.base_dir, experiment_id)

            if not os.path.isdir(experiment_path):
                continue

            try:
                experiment = ExperimentSession(experiment_id, self.base_dir)

                # Filter by status if specified
                if status and experiment.get_status() != status:
                    continue

                experiments.append(experiment)

            except Exception as e:
                print(f"Warning: Could not load experiment {experiment_id}: {e}")

        # Sort by last_updated (most recent first)
        experiments.sort(
            key=lambda e: e.metadata.get('last_updated', ''),
            reverse=True
        )

        return experiments

    def select_experiment_interactive(self) -> Optional[ExperimentSession]:
        """
        Interactive experiment selection.

        Returns:
            Selected ExperimentSession or None
        """
        print("\n" + "=" * 70)
        print("TossingBot Experiment Manager")
        print("=" * 70)

        # List existing experiments
        experiments = self.list_experiments()

        if experiments:
            print("\nAvailable experiments:")
            for i, exp in enumerate(experiments, 1):
                config = exp.load_config() if os.path.exists(exp.config_path) else {}
                name = config.get('name', exp.experiment_id)
                desc = config.get('description', 'No description')
                status = exp.get_status()
                created = exp.metadata.get('created_at', 'unknown')[:10]

                print(f"  {i}) {name} [{status}]")
                print(f"     {desc}")
                print(f"     Created: {created}")
                print()
        else:
            print("\nNo experiments found.")
            print()

        # Get user choice
        print("Options:")
        if experiments:
            print(f"  [1-{len(experiments)}] Select existing experiment")
        print("  [n] Create new experiment")
        print("  [q] Quit")
        print()

        choice = input("Choice: ").strip().lower()

        if choice == 'q':
            return None
        elif choice == 'n':
            config_path = input("Enter path to config YAML file: ").strip()
            if not config_path:
                print("No config file provided.")
                return None
            try:
                return self.create_experiment_from_file(config_path)
            except Exception as e:
                print(f"Error creating experiment: {e}")
                return None
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(experiments):
                    return experiments[idx]
                else:
                    print("Invalid choice.")
                    return None
            except ValueError:
                print("Invalid choice.")
                return None

    def delete_experiment(self, experiment_id: str, force: bool = False):
        """
        Delete an experiment.

        Args:
            experiment_id: Experiment to delete
            force: Skip confirmation if True
        """
        import shutil

        experiment_path = os.path.join(self.base_dir, experiment_id)

        if not os.path.exists(experiment_path):
            print(f"Experiment not found: {experiment_id}")
            return

        if not force:
            response = input(f"Delete experiment '{experiment_id}'? (yes/no): ")
            if response.lower() not in ['yes', 'y']:
                print("Cancelled.")
                return

        shutil.rmtree(experiment_path)
        print(f"Deleted experiment: {experiment_id}")

    def archive_experiment(self, experiment_id: str):
        """Archive an experiment by marking it inactive"""
        experiment = self.load_experiment(experiment_id)
        if experiment:
            experiment.set_status('archived')
            print(f"Archived experiment: {experiment_id}")

    # ========== Validation ==========

    def _validate_config(self, config: Dict[str, Any]):
        """
        Validate experiment configuration.

        Args:
            config: Configuration dictionary

        Raises:
            ValueError: If config is invalid
        """
        # Check for required fields
        if 'train' not in config:
            raise ValueError("Config must include 'train' section")

        train_config = config['train']
        if 'objects' not in train_config:
            raise ValueError("Train config must include 'objects' list")

        if not isinstance(train_config['objects'], list) or len(train_config['objects']) == 0:
            raise ValueError("Train objects must be a non-empty list")

        # Validate eval configs
        if 'evals' in config:
            evals = config['evals']
            if not isinstance(evals, dict):
                raise ValueError("'evals' must be a dictionary of eval phase configs")

            if len(evals) == 0:
                raise ValueError("If 'evals' is provided, it must contain at least one eval phase")

            for eval_name, eval_config in evals.items():
                if not isinstance(eval_config, dict):
                    raise ValueError(f"Eval phase '{eval_name}' config must be a dictionary")

                if 'objects' not in eval_config:
                    raise ValueError(f"Eval phase '{eval_name}' must include 'objects' list")

                if not isinstance(eval_config['objects'], list) or len(eval_config['objects']) == 0:
                    raise ValueError(f"Eval phase '{eval_name}' objects must be a non-empty list")

                # Validate checkpoint name if provided
                checkpoint = eval_config.get('checkpoint', 'best')
                valid_checkpoints = ['best', 'latest']
                if checkpoint not in valid_checkpoints and not checkpoint.startswith('step_'):
                    print(f"Warning: Eval phase '{eval_name}' has unusual checkpoint: {checkpoint}")

    # ========== Info Display ==========

    def print_experiment_info(self, experiment: ExperimentSession, verbose: bool = False):
        """
        Print detailed experiment information.

        Args:
            experiment: ExperimentSession to display
            verbose: Show detailed information
        """
        summary = experiment.get_summary()

        print(f"\n{'=' * 70}")
        print(f"EXPERIMENT: {summary['name']}")
        print(f"{'=' * 70}")
        print(f"ID: {summary['experiment_id']}")
        print(f"Status: {summary['status']}")
        print(f"Description: {summary['description']}")
        print(f"Created: {summary['created_at']}")
        print(f"Last Updated: {summary['last_updated']}")
        print()

        # Training info
        print("TRAINING PHASE:")
        train_info = summary['training']
        print(f"  Completed: {train_info['completed']}")
        if train_info['completed']:
            stats = train_info['stats']
            print(f"  Steps: {stats.get('total_steps', 0)}")
            print(f"  Episodes: {stats.get('total_episodes', 0)}")
            print(f"  Success Rate: {stats.get('success_rate', 0.0) * 100:.1f}%")
            print(f"  Buffer Size: {stats.get('buffer_size', 0)}")
            if stats.get('best_checkpoint'):
                print(f"  Best Checkpoint: {stats['best_checkpoint']}")
        print()

        # Evaluation info
        eval_info = summary['evaluations']
        print(f"EVALUATION PHASES: ({len(eval_info['defined'])} defined, {len(eval_info['completed'])} completed)")

        if eval_info['defined']:
            config = experiment.load_config()
            evals_config = config.get('evals', {})

            for eval_name in eval_info['defined']:
                is_completed = eval_name in eval_info['completed']
                status_mark = "✓" if is_completed else "○"

                eval_cfg = evals_config.get(eval_name, {})
                objects = eval_cfg.get('objects', [])
                episodes = eval_cfg.get('episodes', 0)

                print(f"  {status_mark} {eval_name}")
                print(f"      Objects: {', '.join(objects)}")
                print(f"      Episodes: {episodes}")

                if is_completed and verbose:
                    stats = eval_info['stats'].get(eval_name, {})
                    if stats:
                        print(f"      Success Rate: {stats.get('success_rate', 0.0) * 100:.1f}%")
                        print(f"      Attempts: {stats.get('attempts', 0)}")
        else:
            print("  No evaluation phases defined")

        print(f"{'=' * 70}\n")

    def compare_experiments(self, experiment_ids: List[str], eval_name: str = None):
        """
        Compare multiple experiments.

        Args:
            experiment_ids: List of experiment IDs to compare
            eval_name: Specific eval phase to compare (optional)
        """
        if len(experiment_ids) < 2:
            print("Error: Need at least 2 experiments to compare")
            return

        # Load experiments
        experiments = []
        for exp_id in experiment_ids:
            exp = self.load_experiment(exp_id)
            if exp is None:
                print(f"Error: Experiment not found: {exp_id}")
                return
            experiments.append(exp)

        print(f"\n{'=' * 70}")
        print(f"EXPERIMENT COMPARISON")
        print(f"{'=' * 70}\n")

        # Header
        header = "Metric".ljust(30)
        for exp in experiments:
            config = exp.load_config() if os.path.exists(exp.config_path) else {}
            name = config.get('name', exp.experiment_id)[:15]
            header += name.ljust(20)
        print(header)
        print("-" * 70)

        # Training metrics
        print("\nTRAINING METRICS:")
        metrics = [
            ('Total Steps', lambda e: e.metadata.get('training_stats', {}).get('total_steps', 0)),
            ('Success Rate', lambda e: f"{e.metadata.get('training_stats', {}).get('success_rate', 0.0) * 100:.1f}%"),
            ('Buffer Size', lambda e: e.metadata.get('training_stats', {}).get('buffer_size', 0)),
        ]

        for label, getter in metrics:
            row = label.ljust(30)
            for exp in experiments:
                value = getter(exp)
                row += str(value).ljust(20)
            print(row)

        # Evaluation metrics
        if eval_name:
            print(f"\nEVALUATION METRICS (phase: {eval_name}):")
            eval_metrics = [
                ('Success Rate', lambda e: f"{e.metadata.get('evaluation_stats', {}).get(eval_name, {}).get('success_rate', 0.0) * 100:.1f}%"),
                ('Attempts', lambda e: e.metadata.get('evaluation_stats', {}).get(eval_name, {}).get('attempts', 0)),
            ]

            for label, getter in eval_metrics:
                row = label.ljust(30)
                for exp in experiments:
                    value = getter(exp)
                    row += str(value).ljust(20)
                print(row)

        print(f"\n{'=' * 70}\n")
