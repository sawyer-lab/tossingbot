"""
Session Manager for TossingBot Training
Manages training and demo sessions with isolated directories and metadata
"""
import os
import json
import datetime
from typing import List, Optional, Dict, Any
from pathlib import Path


class Session:
    """Represents a training or demo session"""
    
    def __init__(self, session_id: str, mode: str, base_dir: str = "sessions", name: Optional[str] = None):
        """
        Args:
            session_id: Unique session identifier
            mode: 'training' or 'demo'
            base_dir: Base directory for all sessions
            name: Optional human-readable name
        """
        self.session_id = session_id
        self.mode = mode
        self.base_dir = base_dir
        self.name = name or session_id
        
        # Paths
        self.session_dir = os.path.join(base_dir, mode, session_id)
        self.metadata_path = os.path.join(self.session_dir, "metadata.json")
        self.hyperparams_path = os.path.join(self.session_dir, "hyperparameters.json")
        
        # Subdirectories
        self.checkpoint_dir = os.path.join(self.session_dir, "checkpoints")
        self.buffer_dir = os.path.join(self.session_dir, "buffer")
        self.logs_dir = os.path.join(self.session_dir, "logs")
        self.analysis_dir = os.path.join(self.session_dir, "analysis")
        
        # Create directory structure
        self._create_directories()
        
        # Load or initialize metadata
        if os.path.exists(self.metadata_path):
            self.metadata = self.load_metadata()
        else:
            self.metadata = self._initialize_metadata()
    
    def _create_directories(self):
        """Create session directory structure"""
        os.makedirs(self.checkpoint_dir, exist_ok=True)
        os.makedirs(self.buffer_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        os.makedirs(self.analysis_dir, exist_ok=True)
    
    def _initialize_metadata(self) -> Dict[str, Any]:
        """Initialize metadata for new session"""
        now = datetime.datetime.now().isoformat()
        return {
            'session_id': self.session_id,
            'session_name': self.name,
            'mode': self.mode,
            'created_at': now,
            'last_run': now,
            'last_updated': now,
            'status': 'active',
            'total_steps': 0,
            'total_episodes': 0,
            'success_rate': 0.0,
            'buffer_size': 0,
            'checkpoints': [],
            'best_checkpoint': None,
            'hyperparameters': {}
        }
    
    def load_metadata(self) -> Dict[str, Any]:
        """Load session metadata from disk"""
        try:
            with open(self.metadata_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Warning: Could not load metadata: {e}")
            return self._initialize_metadata()
    
    def save_metadata(self):
        """Save session metadata to disk"""
        self.metadata['last_updated'] = datetime.datetime.now().isoformat()
        try:
            with open(self.metadata_path, 'w') as f:
                json.dump(self.metadata, f, indent=2)
        except Exception as e:
            print(f"Warning: Could not save metadata: {e}")
    
    def update_metadata(self, **kwargs):
        """Update metadata fields"""
        self.metadata.update(kwargs)
        self.metadata['last_run'] = datetime.datetime.now().isoformat()
        self.save_metadata()
    
    def get_checkpoint_path(self, checkpoint_name: str = "checkpoint_latest.pth") -> str:
        """Get path to a checkpoint file"""
        return os.path.join(self.checkpoint_dir, checkpoint_name)
    
    def get_buffer_path(self) -> str:
        """Get path to replay buffer"""
        return os.path.join(self.buffer_dir, "replay_buffer.pkl")
    
    def get_log_path(self) -> str:
        """Get path to training log"""
        return os.path.join(self.logs_dir, "training_log.jsonl")
    
    def save_hyperparameters(self, hyperparams: Dict[str, Any]):
        """Save hyperparameters to session"""
        try:
            with open(self.hyperparams_path, 'w') as f:
                json.dump(hyperparams, f, indent=2)
            self.metadata['hyperparameters'] = hyperparams
            self.save_metadata()
        except Exception as e:
            print(f"Warning: Could not save hyperparameters: {e}")
    
    def load_hyperparameters(self) -> Optional[Dict[str, Any]]:
        """Load hyperparameters from session"""
        if os.path.exists(self.hyperparams_path):
            try:
                with open(self.hyperparams_path, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Warning: Could not load hyperparameters: {e}")
        return None
    
    def __str__(self):
        """String representation of session"""
        steps = self.metadata.get('total_steps', 0)
        success = self.metadata.get('success_rate', 0.0) * 100
        return f"{self.name} ({steps} steps, {success:.1f}% success)"


class SessionManager:
    """Manages training and demo sessions"""
    
    def __init__(self, base_dir: str = "sessions"):
        """
        Args:
            base_dir: Base directory for all sessions
        """
        self.base_dir = base_dir
        self.training_dir = os.path.join(base_dir, "training")
        self.demo_dir = os.path.join(base_dir, "demo")
        
        # Create base directories
        os.makedirs(self.training_dir, exist_ok=True)
        os.makedirs(self.demo_dir, exist_ok=True)
    
    def list_sessions(self, mode: str = "training") -> List[Session]:
        """
        List all sessions of a given mode.
        
        Args:
            mode: 'training' or 'demo'
        
        Returns:
            List of Session objects
        """
        session_dir = self.training_dir if mode == "training" else self.demo_dir
        sessions = []
        
        if not os.path.exists(session_dir):
            return sessions
        
        for session_id in os.listdir(session_dir):
            session_path = os.path.join(session_dir, session_id)
            if os.path.isdir(session_path):
                try:
                    session = Session(session_id, mode, self.base_dir)
                    sessions.append(session)
                except Exception as e:
                    print(f"Warning: Could not load session {session_id}: {e}")
        
        # Sort by last_run (most recent first)
        sessions.sort(key=lambda s: s.metadata.get('last_run', ''), reverse=True)
        return sessions
    
    def create_session(self, mode: str = "training", name: Optional[str] = None) -> Session:
        """
        Create a new session.
        
        Args:
            mode: 'training' or 'demo'
            name: Optional human-readable name
        
        Returns:
            New Session object
        """
        # Generate session ID
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        if name:
            session_id = f"session_{name}"
        else:
            session_id = f"session_{timestamp}"
        
        # Create session
        session = Session(session_id, mode, self.base_dir, name=name)
        print(f"Created new session: {session_id}")
        return session
    
    def load_session(self, session_id: str, mode: str = "training") -> Optional[Session]:
        """
        Load an existing session.
        
        Args:
            session_id: Session identifier
            mode: 'training' or 'demo'
        
        Returns:
            Session object or None if not found
        """
        session_path = os.path.join(self.base_dir, mode, session_id)
        if not os.path.exists(session_path):
            print(f"Session not found: {session_id}")
            return None
        
        try:
            session = Session(session_id, mode, self.base_dir)
            print(f"Loaded session: {session_id}")
            return session
        except Exception as e:
            print(f"Error loading session: {e}")
            return None
    
    def select_session_interactive(self, mode: str = "training") -> Session:
        """
        Interactive session selection.
        
        Args:
            mode: 'training' or 'demo'
        
        Returns:
            Selected or newly created Session
        """
        print("\n" + "=" * 60)
        print("TossingBot Session Manager")
        print("=" * 60)
        print(f"Mode: {mode.capitalize()}")
        print()
        
        # List existing sessions
        sessions = self.list_sessions(mode)
        
        if sessions:
            print("Available sessions:")
            for i, session in enumerate(sessions, 1):
                created = session.metadata.get('created_at', 'unknown')[:10]
                last_run = session.metadata.get('last_run', 'unknown')[:10]
                print(f"  {i}) {session} - Created: {created}, Last run: {last_run}")
            print()
        else:
            print("No existing sessions found.")
            print()
        
        # Get user choice
        if mode == "training":
            print("Options:")
            if sessions:
                print(f"  [1-{len(sessions)}] Continue existing session")
            print("  [n] Create new session")
            print("  [q] Quit")
            print()
            
            choice = input("Choice: ").strip().lower()
            
            if choice == 'q':
                print("Exiting...")
                exit(0)
            elif choice == 'n':
                name = input("\nEnter session name (or press Enter for auto-generated): ").strip()
                name = name if name else None
                return self.create_session(mode, name)
            else:
                try:
                    idx = int(choice) - 1
                    if 0 <= idx < len(sessions):
                        return sessions[idx]
                    else:
                        print("Invalid choice. Creating new session.")
                        return self.create_session(mode)
                except ValueError:
                    print("Invalid choice. Creating new session.")
                    return self.create_session(mode)
        
        elif mode == "demo":
            if not sessions:
                print("No training sessions available for demo.")
                print("Please train a model first.")
                exit(1)
            
            print("Select training session to demo:")
            print()
            
            choice = input("Session [1-{}] or [q]uit: ".format(len(sessions))).strip().lower()
            
            if choice == 'q':
                print("Exiting...")
                exit(0)
            
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(sessions):
                    training_session = sessions[idx]
                    
                    # Ask for checkpoint
                    print("\nWhich checkpoint to use:")
                    print("  [b] Best checkpoint (recommended)")
                    print("  [l] Latest checkpoint")
                    ckpt_choice = input("Choice [b/l]: ").strip().lower() or 'b'
                    
                    # Create demo session
                    demo_session = self.create_session("demo")
                    demo_session.metadata['source_training_session'] = training_session.session_id
                    demo_session.metadata['source_checkpoint'] = 'best' if ckpt_choice == 'b' else 'latest'
                    demo_session.save_metadata()
                    
                    return demo_session, training_session
                else:
                    print("Invalid choice. Exiting.")
                    exit(1)
            except ValueError:
                print("Invalid choice. Exiting.")
                exit(1)
        
        return self.create_session(mode)
    
    def archive_session(self, session_id: str, mode: str = "training"):
        """Archive a session by marking it inactive"""
        session = self.load_session(session_id, mode)
        if session:
            session.metadata['status'] = 'archived'
            session.save_metadata()
            print(f"Archived session: {session_id}")
    
    def delete_session(self, session_id: str, mode: str = "training"):
        """Delete a session (use with caution)"""
        import shutil
        session_path = os.path.join(self.base_dir, mode, session_id)
        if os.path.exists(session_path):
            shutil.rmtree(session_path)
            print(f"Deleted session: {session_id}")
        else:
            print(f"Session not found: {session_id}")
