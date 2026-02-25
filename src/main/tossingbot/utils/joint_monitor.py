import time
import threading
import numpy as np
from typing import List, Dict, Optional
from collections import defaultdict

class JointMonitor:
    """
    Monitor for tracking robot joint positions and velocities using ZMQ SUB socket.
    This avoids interfering with the command (REQ) socket during trajectory execution.
    """
    def __init__(self, robot, hz: float = 100.0):
        self.robot = robot
        self.hz = hz
        self.dt = 1.0 / hz
        
        self._recording = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        
        # Data storage
        self.data = defaultdict(list)
        
    def start_recording(self):
        """Start a background thread to poll state from the SUB socket."""
        # Initialize the subscription
        try:
            self.robot._client.subscribe_to_state()
            # Give ZMQ a moment to establish the connection and start receiving
            time.sleep(0.1)
        except Exception as e:
            print(f"Warning: Could not subscribe to state: {e}")

        with self._lock:
            if self._recording:
                return
            self._recording = True
            self.data = defaultdict(list)
            self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._thread.start()
            
    def stop_recording(self):
        """Stop the background polling thread."""
        with self._lock:
            self._recording = False
        if self._thread:
            self._thread.join(timeout=1.0)
            
    def _monitor_loop(self):
        start_time = time.time()
        while True:
            with self._lock:
                if not self._recording:
                    break
            
            try:
                # Use non-blocking SUB socket poll
                state = self.robot._client.get_latest_state()
                
                # Update keys to match the actual ZMQ broadcast
                if state and 'robot' in state:
                    robot_state = state['robot']
                    t = time.time() - start_time
                    q = robot_state.get('joint_angles', [])
                    qd = robot_state.get('joint_velocities', [])
                    endpoint_pose = robot_state.get('endpoint_pose', {})
                    endpoint_vel = robot_state.get('endpoint_velocity', {})
                    
                    if q and qd and endpoint_pose and endpoint_vel:
                        if not self.data['time']: # Only print once at first sample
                            print(f"JointMonitor: First sample collected at {t:.3f}s")
                        with self._lock:
                            self.data['time'].append(t)
                            self.data['q'].append(q)
                            self.data['qd'].append(qd)
                            self.data['endpoint_pos'].append(endpoint_pose.get('position', [0,0,0]))
                            self.data['endpoint_vel'].append(endpoint_vel.get('linear', [0,0,0]))
            except Exception as e:
                pass
            
            # Sleep to maintain frequency
            time.sleep(self.dt)

    def get_data(self) -> Dict[str, np.ndarray]:
        """Returns recorded data as numpy arrays."""
        with self._lock:
            if not self.data['time']:
                return {
                    'time': np.array([]), 
                    'q': np.array([]).reshape(0, 7), 
                    'qd': np.array([]).reshape(0, 7),
                    'endpoint_pos': np.array([]).reshape(0, 3),
                    'endpoint_vel': np.array([]).reshape(0, 3)
                }
            return {
                'time': np.array(self.data['time']),
                'q': np.array(self.data['q']),
                'qd': np.array(self.data['qd']),
                'endpoint_pos': np.array(self.data['endpoint_pos']),
                'endpoint_vel': np.array(self.data['endpoint_vel'])
            }

    def save_to_csv(self, filename: str):
        """Save recorded data to a CSV file."""
        import csv
        data = self.get_data()
        if len(data['time']) == 0:
            print("No data recorded to save.")
            return
            
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Header
            header = ['time'] + [f'q{i}' for i in range(data['q'].shape[1])] + \
                               [f'qd{i}' for i in range(data['qd'].shape[1])] + \
                               ['x', 'y', 'z', 'vx', 'vy', 'vz']
            writer.writerow(header)
            
            for i in range(len(data['time'])):
                row = [data['time'][i]] + list(data['q'][i]) + list(data['qd'][i]) + \
                      list(data['endpoint_pos'][i]) + list(data['endpoint_vel'][i])
                writer.writerow(row)
        print(f"Joint data saved to {filename} ({len(data['time'])} samples)")
