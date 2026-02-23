"""
Experiment Session Management for TossingBot
Manages unified experiment sessions: one training + multiple evaluation phases
"""
import os
import yaml
import json
import datetime
from typing import List, Optional, Dict, Any
from pathlib import Path


class ExperimentSession:
    """
    Unified experiment session: one training phase + multiple evaluation phases

    Structure:
        experiments/<experiment_id>/
        ├── config.yaml          # Declarative experiment definition
        ├── metadata.json        # Runtime state and results
        ├── train/               # Training phase artifacts
        │   ├── checkpoints/
        │   ├── buffer/
        │   └── logs/
        ├── evals/               # Multiple evaluation phases
        │   ├── seen/
        │   │   └── logs/
        │   ├── unseen/
        │   │   └── logs/
        │   └── ...
        └── analysis/            # Combined analysis results
    """

    def __init__(self, experiment_id: str, base_dir: str = "sessions/experiments"):
        """
        Initialize experiment session.

        Args:
            experiment_id: Unique experiment identifier
            base_dir: Base directory for all experiments
        """
        self.experiment_id = experiment_id
        self.base_dir = base_dir
        self.experiment_dir = os.path.join(base_dir, experiment_id)

        # Main configuration and metadata files
        self.config_path = os.path.join(self.experiment_dir, "config.yaml")
        self.metadata_path = os.path.join(self.experiment_dir, "metadata.json")

        # Phase directories
        self.train_dir = os.path.join(self.experiment_dir, "train")
        self.evals_dir = os.path.join(self.experiment_dir, "evals")
        self.analysis_dir = os.path.join(self.experiment_dir, "analysis")

        # Create directory structure
        self._create_directories()

        # Load or initialize metadata
        if os.path.exists(self.metadata_path):
            self.metadata = self._load_metadata()
        else:
            self.metadata = self._initialize_metadata()

    def _create_directories(self):
        """Create experiment directory structure"""
        # Training directories
        os.makedirs(os.path.join(self.train_dir, "checkpoints"), exist_ok=True)
        os.makedirs(os.path.join(self.train_dir, "buffer"), exist_ok=True)
        os.makedirs(os.path.join(self.train_dir, "logs"), exist_ok=True)

        # Evals base directory
        os.makedirs(self.evals_dir, exist_ok=True)

        # Analysis directory
        os.makedirs(self.analysis_dir, exist_ok=True)

    def _initialize_metadata(self) -> Dict[str, Any]:
        """Initialize metadata for new experiment"""
        now = datetime.datetime.now().isoformat()
        return {
            'experiment_id': self.experiment_id,
            'created_at': now,
            'last_updated': now,
            'status': 'created',  # created, training, evaluating, completed, archived
            'phases_completed': {
                'train': False,
                'evals': {}  # eval_name -> True/False
            },
            'training_stats': {
                'total_steps': 0,
                'total_episodes': 0,
                'success_rate': 0.0,
                'buffer_size': 0,
                'best_checkpoint': None,
                'last_checkpoint': None
            },
            'evaluation_stats': {}  # eval_name -> stats dict
        }

    def _load_metadata(self) -> Dict[str, Any]:
        """Load experiment metadata from disk"""
        try:
            with open(self.metadata_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Warning: Could not load metadata: {e}")
            return self._initialize_metadata()

    def save_metadata(self):
        """Save experiment metadata to disk"""
        self.metadata['last_updated'] = datetime.datetime.now().isoformat()
        try:
            with open(self.metadata_path, 'w') as f:
                json.dump(self.metadata, f, indent=2)
        except Exception as e:
            print(f"Warning: Could not save metadata: {e}")

    def load_config(self) -> Dict[str, Any]:
        """Load experiment configuration from YAML"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise ValueError(f"Could not load config: {e}")

    def save_config(self, config: Dict[str, Any]):
        """Save experiment configuration to YAML"""
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            raise ValueError(f"Could not save config: {e}")

    # ========== Path Getters ==========

    def get_train_log_path(self) -> str:
        """Get path to training log"""
        return os.path.join(self.train_dir, "logs", "training_log.jsonl")

    def get_eval_log_path(self, eval_name: str) -> str:
        """Get log path for specific eval phase"""
        return os.path.join(self.evals_dir, eval_name, "logs", "evaluation_log.jsonl")

    def get_eval_dir(self, eval_name: str) -> str:
        """Get directory for specific eval phase"""
        return os.path.join(self.evals_dir, eval_name)

    def get_checkpoint_dir(self) -> str:
        """Get training checkpoints directory"""
        return os.path.join(self.train_dir, "checkpoints")

    def get_checkpoint_path(self, checkpoint_name: str = "checkpoint_best.pth") -> str:
        """Get path to specific checkpoint"""
        return os.path.join(self.get_checkpoint_dir(), checkpoint_name)

    def get_buffer_path(self) -> str:
        """Get path to replay buffer"""
        return os.path.join(self.train_dir, "buffer", "replay_buffer.pkl")

    def get_analysis_plots_dir(self, subdir: str = None) -> str:
        """Get analysis plots directory (optionally with subdirectory)"""
        if subdir:
            return os.path.join(self.analysis_dir, subdir)
        return self.analysis_dir

    # ========== Eval Phase Management ==========

    def list_eval_phases(self) -> List[str]:
        """List all eval phases defined in config"""
        try:
            config = self.load_config()
            return list(config.get('evals', {}).keys())
        except Exception:
            return []

    def get_eval_config(self, eval_name: str) -> Dict[str, Any]:
        """Get configuration for specific eval phase"""
        config = self.load_config()
        evals = config.get('evals', {})

        if eval_name not in evals:
            raise ValueError(f"Eval phase '{eval_name}' not defined in config. "
                           f"Available: {list(evals.keys())}")

        return evals[eval_name]

    def create_eval_phase_dirs(self, eval_name: str):
        """Create directories for a new eval phase"""
        eval_dir = self.get_eval_dir(eval_name)
        os.makedirs(os.path.join(eval_dir, "logs"), exist_ok=True)

    def mark_eval_completed(self, eval_name: str, stats: Dict[str, Any] = None):
        """Mark eval phase as completed and save stats"""
        if 'phases_completed' not in self.metadata:
            self.metadata['phases_completed'] = {'train': False, 'evals': {}}

        self.metadata['phases_completed']['evals'][eval_name] = True

        if stats:
            if 'evaluation_stats' not in self.metadata:
                self.metadata['evaluation_stats'] = {}
            self.metadata['evaluation_stats'][eval_name] = stats

        self.save_metadata()

    def is_eval_completed(self, eval_name: str) -> bool:
        """Check if eval phase is completed"""
        return self.metadata.get('phases_completed', {}).get('evals', {}).get(eval_name, False)

    # ========== Training Phase Management ==========

    def mark_training_completed(self, stats: Dict[str, Any] = None):
        """Mark training phase as completed"""
        if 'phases_completed' not in self.metadata:
            self.metadata['phases_completed'] = {'train': False, 'evals': {}}

        self.metadata['phases_completed']['train'] = True

        if stats:
            self.metadata['training_stats'].update(stats)

        self.save_metadata()

    def is_training_completed(self) -> bool:
        """Check if training phase is completed"""
        return self.metadata.get('phases_completed', {}).get('train', False)

    def update_training_stats(self, **kwargs):
        """Update training statistics"""
        self.metadata['training_stats'].update(kwargs)
        self.save_metadata()

    # ========== Status Management ==========

    def set_status(self, status: str):
        """
        Set experiment status

        Valid statuses: created, training, evaluating, completed, archived
        """
        valid_statuses = ['created', 'training', 'evaluating', 'completed', 'archived']
        if status not in valid_statuses:
            raise ValueError(f"Invalid status: {status}. Valid: {valid_statuses}")

        self.metadata['status'] = status
        self.save_metadata()

    def get_status(self) -> str:
        """Get current experiment status"""
        return self.metadata.get('status', 'unknown')

    # ========== String Representation ==========

    def __str__(self):
        """String representation of experiment"""
        config = self.load_config() if os.path.exists(self.config_path) else {}
        name = config.get('name', self.experiment_id)
        status = self.get_status()

        train_stats = self.metadata.get('training_stats', {})
        steps = train_stats.get('total_steps', 0)
        success_rate = train_stats.get('success_rate', 0.0) * 100

        eval_count = len(self.metadata.get('phases_completed', {}).get('evals', {}))

        return (f"{name} [{status}] "
                f"(Train: {steps} steps, {success_rate:.1f}% success, "
                f"Evals: {eval_count} completed)")

    def get_summary(self) -> Dict[str, Any]:
        """Get comprehensive summary of experiment"""
        config = self.load_config() if os.path.exists(self.config_path) else {}

        return {
            'experiment_id': self.experiment_id,
            'name': config.get('name', self.experiment_id),
            'description': config.get('description', ''),
            'status': self.get_status(),
            'created_at': self.metadata.get('created_at'),
            'last_updated': self.metadata.get('last_updated'),
            'training': {
                'completed': self.is_training_completed(),
                'stats': self.metadata.get('training_stats', {})
            },
            'evaluations': {
                'defined': self.list_eval_phases(),
                'completed': [k for k, v in self.metadata.get('phases_completed', {}).get('evals', {}).items() if v],
                'stats': self.metadata.get('evaluation_stats', {})
            }
        }
