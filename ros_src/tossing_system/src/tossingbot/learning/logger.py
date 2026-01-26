"""
Training Logger for TossingBot
Logs training data in JSON Lines format for analysis
"""
import os
import json
import datetime
from collections import defaultdict


class TrainingLogger:
    """Logs training steps and episodes for later analysis"""
    
    def __init__(self, log_dir, experiment_name=None):
        """
        Args:
            log_dir: Directory to save logs
            experiment_name: Optional name for this training run
        """
        self.log_dir = log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Generate log filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        if experiment_name:
            self.log_filename = f"training_log_{experiment_name}_{timestamp}.jsonl"
        else:
            self.log_filename = f"training_log_{timestamp}.jsonl"
        
        self.log_path = os.path.join(log_dir, self.log_filename)
        self.summary_path = os.path.join(log_dir, f"training_summary_{timestamp}.json")
        
        # Buffered writing
        self.buffer = []
        self.buffer_size = 10  # Flush every 10 steps
        
        # Statistics tracking
        self.stats = {
            'total_steps': 0,
            'total_episodes': 0,
            'successes': 0,
            'failures': 0,
            'total_reward': 0.0,
            'total_loss': 0.0,
            'per_object_stats': defaultdict(lambda: {'attempts': 0, 'successes': 0}),
            'start_time': datetime.datetime.now().isoformat(),
        }
        
        print(f"Logging to: {self.log_path}")
    
    def log_step(self, step_data):
        """
        Log a single training step.
        
        Expected keys in step_data:
        - step: int
        - episode: int (optional)
        - object_name: str
        - object_pose: dict with 'position' and 'orientation'
        - predicted_grasp: dict with 'u', 'v', 'rotation', 'confidence'
        - action_type: str ('EXPLORE' or 'EXPLOIT')
        - epsilon: float
        - reward: float
        - loss: float
        - success: bool
        - timestamp: str (auto-added if not present)
        """
        # Add timestamp if not present
        if 'timestamp' not in step_data:
            step_data['timestamp'] = datetime.datetime.now().isoformat()
        
        # Update statistics
        self.stats['total_steps'] += 1
        self.stats['total_reward'] += step_data.get('reward', 0.0)
        self.stats['total_loss'] += step_data.get('loss', 0.0)
        
        if step_data.get('success', False):
            self.stats['successes'] += 1
        else:
            self.stats['failures'] += 1
        
        # Per-object statistics
        obj_name = step_data.get('object_name', 'unknown')
        if obj_name != 'unknown':
            self.stats['per_object_stats'][obj_name]['attempts'] += 1
            if step_data.get('success', False):
                self.stats['per_object_stats'][obj_name]['successes'] += 1
        
        # Buffer the log entry
        self.buffer.append(step_data)
        
        # Flush if buffer is full
        if len(self.buffer) >= self.buffer_size:
            self.flush()
    
    def log_episode_end(self, episode_summary):
        """
        Log episode-level statistics.
        
        Expected keys:
        - episode: int
        - steps_in_episode: int
        - objects_picked: int
        - total_objects: int
        - success_rate: float
        """
        self.stats['total_episodes'] += 1
        
        # Write as special episode marker
        episode_summary['event_type'] = 'episode_end'
        episode_summary['timestamp'] = datetime.datetime.now().isoformat()
        
        self.buffer.append(episode_summary)
        self.flush()  # Always flush episode summaries
    
    def flush(self):
        """Write buffered logs to disk"""
        if not self.buffer:
            return
        
        with open(self.log_path, 'a') as f:
            for entry in self.buffer:
                f.write(json.dumps(entry) + '\n')
        
        self.buffer.clear()
    
    def save_summary(self):
        """Save aggregated statistics"""
        # Compute derived statistics
        if self.stats['total_steps'] > 0:
            self.stats['success_rate'] = self.stats['successes'] / self.stats['total_steps']
            self.stats['avg_loss'] = self.stats['total_loss'] / self.stats['total_steps']
            self.stats['avg_reward'] = self.stats['total_reward'] / self.stats['total_steps']
        
        # Per-object success rates
        for obj_name, obj_stats in self.stats['per_object_stats'].items():
            if obj_stats['attempts'] > 0:
                obj_stats['success_rate'] = obj_stats['successes'] / obj_stats['attempts']
        
        self.stats['end_time'] = datetime.datetime.now().isoformat()
        
        # Save summary
        with open(self.summary_path, 'w') as f:
            json.dumps(self.stats, f, indent=2, default=str)
        
        print(f"\nTraining summary saved to: {self.summary_path}")
        print(f"Total steps: {self.stats['total_steps']}")
        print(f"Success rate: {self.stats.get('success_rate', 0.0)*100:.2f}%")
    
    def close(self):
        """Close logger and flush remaining data"""
        self.flush()
        self.save_summary()


def load_training_log(log_path):
    """
    Load training log from JSONL file.
    Returns list of log entries.
    """
    entries = []
    with open(log_path, 'r') as f:
        for line in f:
            entries.append(json.loads(line))
    return entries
