"""
Velocity Monitor for tracking end-effector and object velocities during tossing.

This class monitors and records velocity data from both the robot end-effector
and objects in the Gazebo simulation, useful for validating physics calculations
and debugging trajectory execution.
"""
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from collections import defaultdict
import threading


class VelocityMonitor:
    """
    Monitors and records velocities of objects and the robot end-effector.

    Subscribes to /gazebo/model_states to track object velocities in simulation.
    Can also track end-effector velocities from a robot interface.
    """

    def __init__(self, model_name="toss_cube"):
        """
        Initialize the velocity monitor.

        Args:
            model_name: Name of the Gazebo model to track (default: "toss_cube")
        """
        self.model_name = model_name
        self._lock = threading.Lock()

        # Storage for time series data
        self._recording = False
        self._ee_data = defaultdict(list)  # {timestamp, vx, vy, vz, v_magnitude}
        self._obj_data = defaultdict(list)  # {timestamp, vx, vy, vz, v_magnitude}

        # Current state
        self._current_obj_velocity = None
        self._current_obj_position = None
        self._model_exists = False

        # Subscribe to model states
        self._sub = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self._callback_model_states,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo(f"VelocityMonitor: Initialized for model '{model_name}'")

    def _callback_model_states(self, msg):
        """Callback for /gazebo/model_states topic."""
        try:
            idx = msg.name.index(self.model_name)
            self._model_exists = True

            # Extract velocity
            twist = msg.twist[idx]
            vx = twist.linear.x
            vy = twist.linear.y
            vz = twist.linear.z
            v_mag = np.sqrt(vx**2 + vy**2 + vz**2)

            # Extract position
            pose = msg.pose[idx]
            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z

            with self._lock:
                self._current_obj_velocity = {'vx': vx, 'vy': vy, 'vz': vz, 'v_mag': v_mag}
                self._current_obj_position = {'x': px, 'y': py, 'z': pz}

                # Record if currently recording
                if self._recording:
                    timestamp = rospy.Time.now().to_sec()
                    self._obj_data['time'].append(timestamp)
                    self._obj_data['vx'].append(vx)
                    self._obj_data['vy'].append(vy)
                    self._obj_data['vz'].append(vz)
                    self._obj_data['v_mag'].append(v_mag)
                    self._obj_data['x'].append(px)
                    self._obj_data['y'].append(py)
                    self._obj_data['z'].append(pz)

        except ValueError:
            # Model not found in list
            self._model_exists = False

    def start_recording(self):
        """Start recording velocity time series data."""
        with self._lock:
            self._recording = True
            self._ee_data = defaultdict(list)
            self._obj_data = defaultdict(list)
        rospy.loginfo("VelocityMonitor: Started recording")

    def stop_recording(self):
        """Stop recording velocity data."""
        with self._lock:
            self._recording = False
        rospy.loginfo("VelocityMonitor: Stopped recording")

    def record_ee_velocity(self, ee_velocity_dict):
        """
        Manually record end-effector velocity.

        Args:
            ee_velocity_dict: Dict with 'linear': [vx, vy, vz], 'angular': [wx, wy, wz]
        """
        if not self._recording:
            return

        vx = ee_velocity_dict['linear'][0]
        vy = ee_velocity_dict['linear'][1]
        vz = ee_velocity_dict['linear'][2]
        v_mag = np.sqrt(vx**2 + vy**2 + vz**2)

        with self._lock:
            timestamp = rospy.Time.now().to_sec()
            self._ee_data['time'].append(timestamp)
            self._ee_data['vx'].append(vx)
            self._ee_data['vy'].append(vy)
            self._ee_data['vz'].append(vz)
            self._ee_data['v_mag'].append(v_mag)

    def get_current_object_velocity(self):
        """
        Get the current object velocity.

        Returns:
            dict with keys 'vx', 'vy', 'vz', 'v_mag' or None if not available
        """
        with self._lock:
            return self._current_obj_velocity.copy() if self._current_obj_velocity else None

    def get_current_object_position(self):
        """
        Get the current object position.

        Returns:
            dict with keys 'x', 'y', 'z' or None if not available
        """
        with self._lock:
            return self._current_obj_position.copy() if self._current_obj_position else None

    def get_recorded_data(self):
        """
        Get all recorded data.

        Returns:
            tuple: (ee_data_dict, obj_data_dict) where each dict contains lists
                  of time series data
        """
        with self._lock:
            ee_copy = {k: list(v) for k, v in self._ee_data.items()}
            obj_copy = {k: list(v) for k, v in self._obj_data.items()}
        return ee_copy, obj_copy

    def get_peak_velocities(self):
        """
        Analyze recorded data to find peak velocities.

        Returns:
            dict with peak velocity information for both EE and object
        """
        ee_data, obj_data = self.get_recorded_data()

        result = {}

        # EE peaks
        if ee_data and len(ee_data.get('v_mag', [])) > 0:
            ee_v_mag = np.array(ee_data['v_mag'])
            ee_peak_idx = np.argmax(ee_v_mag)
            result['ee_peak'] = {
                'time': ee_data['time'][ee_peak_idx],
                'v_mag': ee_v_mag[ee_peak_idx],
                'vx': ee_data['vx'][ee_peak_idx],
                'vy': ee_data['vy'][ee_peak_idx],
                'vz': ee_data['vz'][ee_peak_idx],
            }

        # Object peaks
        if obj_data and len(obj_data.get('v_mag', [])) > 0:
            obj_v_mag = np.array(obj_data['v_mag'])
            obj_peak_idx = np.argmax(obj_v_mag)
            result['obj_peak'] = {
                'time': obj_data['time'][obj_peak_idx],
                'v_mag': obj_v_mag[obj_peak_idx],
                'vx': obj_data['vx'][obj_peak_idx],
                'vy': obj_data['vy'][obj_peak_idx],
                'vz': obj_data['vz'][obj_peak_idx],
                'x': obj_data['x'][obj_peak_idx],
                'y': obj_data['y'][obj_peak_idx],
                'z': obj_data['z'][obj_peak_idx],
            }

        return result

    def print_analysis(self):
        """Print a detailed analysis of recorded velocities."""
        peaks = self.get_peak_velocities()

        print("\n" + "="*60)
        print("VELOCITY MONITOR ANALYSIS")
        print("="*60)

        if 'ee_peak' in peaks:
            ee = peaks['ee_peak']
            print(f"\nEnd-Effector Peak Velocity:")
            print(f"  Magnitude: {ee['v_mag']:.3f} m/s")
            print(f"  Components: vx={ee['vx']:.3f}, vy={ee['vy']:.3f}, vz={ee['vz']:.3f} m/s")
            print(f"  Time: {ee['time']:.3f} s")
        else:
            print("\nEnd-Effector: No data recorded")

        if 'obj_peak' in peaks:
            obj = peaks['obj_peak']
            print(f"\nObject Peak Velocity:")
            print(f"  Magnitude: {obj['v_mag']:.3f} m/s")
            print(f"  Components: vx={obj['vx']:.3f}, vy={obj['vy']:.3f}, vz={obj['vz']:.3f} m/s")
            print(f"  Position: x={obj['x']:.3f}, y={obj['y']:.3f}, z={obj['z']:.3f} m")
            print(f"  Time: {obj['time']:.3f} s")

            # Calculate angle
            v_horizontal = np.sqrt(obj['vx']**2 + obj['vy']**2)
            angle_deg = np.rad2deg(np.arctan2(obj['vz'], v_horizontal))
            print(f"  Release Angle: {angle_deg:.1f}°")
        else:
            print("\nObject: No data recorded")

        print("="*60 + "\n")

    def is_model_present(self):
        """Check if the tracked model exists in the simulation."""
        return self._model_exists
