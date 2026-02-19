"""
ZeroMQ Protocol Implementation

Fastest protocol for robot control (~1-3ms latency).
Uses REQ/REP for commands and SUB for state updates.
"""

import zmq
import time
import cv2
import numpy as np
from typing import List, Dict, Optional


class ZMQClient:
    """
    ZeroMQ client for robot control.

    Uses two socket types:
      - REQ socket: Send commands, get responses
      - SUB socket: Subscribe to state updates (optional, 100Hz)
    """

    def __init__(self, host='localhost', command_port=5555, state_port=5556):
        """
        Initialize ZMQ client.

        Args:
            host: Container hostname or IP
            command_port: Port for command socket (REQ)
            state_port: Port for state updates (SUB)
        """
        self.host = host
        self.command_port = command_port
        self.state_port = state_port

        # Create ZMQ context
        self.context = zmq.Context()

        # REQ socket for commands
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(f"tcp://{host}:{command_port}")

        # SUB socket for state updates (optional - not created by default)
        self.sub_socket = None

        print(f"✓ ZMQ Client connected to {host}:{command_port}")

    def _send_command(self, command: str, **kwargs) -> Dict:
        """
        Send command to server and get response.

        Args:
            command: Command name
            **kwargs: Command arguments

        Returns:
            Response dict from server
        """
        message = {'command': command, **kwargs}
        self.req_socket.send_json(message)

        response = self.req_socket.recv_json()
        return response

    # ===== Robot Commands =====

    def move_to_joints(self, angles: List[float], timeout: float = 30.0) -> bool:
        """Move robot to target joint positions."""
        response = self._send_command('move_robot', angles=angles, timeout=timeout)
        return response.get('status') == 'ok'

    def get_joint_angles(self) -> List[float]:
        """Get current joint angles."""
        response = self._send_command('get_angles')
        if response.get('status') == 'ok':
            return response.get('angles', [])
        return []

    def get_endpoint_pose(self) -> Optional[Dict]:
        """Get end-effector pose."""
        response = self._send_command('get_endpoint_pose')
        if response.get('status') == 'ok':
            return response.get('pose')
        return None

    def get_robot_state(self) -> Dict:
        """Get complete robot state."""
        response = self._send_command('get_robot_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    def get_joint_velocities(self) -> List[float]:
        """Get current joint velocities."""
        response = self._send_command('get_joint_velocities')
        if response.get('status') == 'ok':
            return response.get('velocities', [])
        return []

    def get_endpoint_velocity(self) -> Optional[Dict]:
        """Get end-effector velocity (linear and angular)."""
        response = self._send_command('get_endpoint_velocity')
        if response.get('status') == 'ok':
            return response.get('velocity')
        return None

    def execute_trajectory(self, waypoints: List[Dict], rate_hz: float = 100.0) -> bool:
        """
        Send full trajectory for container-side execution.

        Args:
            waypoints: List of {position, velocity, acceleration} dicts.
            rate_hz: Execution rate in Hz.
        """
        response = self._send_command('execute_trajectory',
                                       waypoints=waypoints,
                                       rate_hz=rate_hz)
        return response.get('status') == 'ok'

    def execute_toss_trajectory(self, Q, Qd, Qdd, release_index=None) -> bool:
        """
        Execute toss trajectory at 100Hz with async gripper release.

        The server runs the 100Hz loop and fires the gripper at release_index
        step inside the container, so the gripper command does not interrupt
        the trajectory.

        Args:
            Q:   (N, 7) joint positions as nested list or numpy array.
            Qd:  (N, 7) joint velocities.
            Qdd: (N, 7) joint accelerations.
            release_index: Step index at which to open the gripper (None = never).

        Returns:
            True if trajectory completed successfully.
        """
        # Convert to nested lists for JSON serialization
        if hasattr(Q, 'tolist'):
            Q = Q.tolist()
        if hasattr(Qd, 'tolist'):
            Qd = Qd.tolist()
        if hasattr(Qdd, 'tolist'):
            Qdd = Qdd.tolist()

        response = self._send_command(
            'execute_toss_trajectory',
            Q=Q, Qd=Qd, Qdd=Qdd,
            release_index=int(release_index) if release_index is not None else None
        )
        return response.get('status') == 'ok'

    # ===== Gripper Commands =====

    def gripper_open(self) -> bool:
        """Open gripper."""
        response = self._send_command('gripper_open')
        return response.get('status') == 'ok'

    def gripper_close(self) -> bool:
        """Close gripper."""
        response = self._send_command('gripper_close')
        return response.get('status') == 'ok'

    def gripper_set_position(self, position: float) -> bool:
        """Set gripper position."""
        response = self._send_command('gripper_set_position', position=position)
        return response.get('status') == 'ok'

    def gripper_get_state(self) -> Dict:
        """Get gripper state."""
        response = self._send_command('gripper_get_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    # ===== Camera Commands =====

    def camera_start(self, camera=None) -> bool:
        """Start camera streaming."""
        kwargs = {}
        if camera is not None:
            kwargs['camera'] = camera
        response = self._send_command('camera_start', **kwargs)
        return response.get('status') == 'ok'

    def camera_stop(self, camera=None) -> bool:
        """Stop camera streaming."""
        kwargs = {}
        if camera is not None:
            kwargs['camera'] = camera
        response = self._send_command('camera_stop', **kwargs)
        return response.get('status') == 'ok'

    def camera_get_image(self, camera=None):
        """Get current camera image as numpy array."""
        kwargs = {}
        if camera is not None:
            kwargs['camera'] = camera
        response = self._send_command('camera_get_image', **kwargs)
        if response.get('status') == 'ok':
            img_hex = response.get('image')
            if img_hex:
                img_bytes = bytes.fromhex(img_hex)
                nparr = np.frombuffer(img_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                return img
        return None

    def camera_get_state(self, camera=None):
        """Get camera state dict."""
        kwargs = {}
        if camera is not None:
            kwargs['camera'] = camera
        response = self._send_command('camera_get_state', **kwargs)
        return response.get('state') if response.get('status') == 'ok' else None

    # ===== Lights Commands =====

    def lights_list(self):
        response = self._send_command('lights_list')
        return response.get('lights', []) if response.get('status') == 'ok' else []

    def lights_set(self, name, on=True):
        response = self._send_command('lights_set', name=name, on=on)
        return response.get('status') == 'ok'

    def lights_get(self, name):
        response = self._send_command('lights_get', name=name)
        return response.get('on', False) if response.get('status') == 'ok' else False

    # ===== Head Commands =====

    def head_pan(self):
        response = self._send_command('head_pan')
        return response.get('angle', 0.0) if response.get('status') == 'ok' else 0.0

    def head_set_pan(self, angle, speed=1.0):
        response = self._send_command('head_set_pan', angle=angle, speed=speed)
        return response.get('status') == 'ok'

    # ===== Head Display Commands =====

    def display_image(self, image):
        _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_hex = buffer.tobytes().hex()
        response = self._send_command('display_image', image=img_hex)
        return response.get('status') == 'ok'

    def display_clear(self):
        response = self._send_command('display_clear')
        return response.get('status') == 'ok'

    # ===== Robot Enable Commands =====

    def robot_enable(self):
        response = self._send_command('robot_enable')
        return response.get('status') == 'ok'

    def robot_disable(self):
        response = self._send_command('robot_disable')
        return response.get('status') == 'ok'

    def robot_reset(self):
        response = self._send_command('robot_reset')
        return response.get('status') == 'ok'

    def robot_stop(self):
        response = self._send_command('robot_stop')
        return response.get('status') == 'ok'

    def robot_state(self):
        response = self._send_command('robot_state')
        return response.get('state', {}) if response.get('status') == 'ok' else {}

    # ===== Robot Params Commands =====

    def params_robot_name(self):
        response = self._send_command('params_robot_name')
        return response.get('name') if response.get('status') == 'ok' else None

    def params_limb_names(self):
        response = self._send_command('params_limb_names')
        return response.get('limbs', []) if response.get('status') == 'ok' else []

    def params_joint_names(self, limb='right'):
        response = self._send_command('params_joint_names', limb=limb)
        return response.get('joints', []) if response.get('status') == 'ok' else []

    def params_camera_names(self):
        response = self._send_command('params_camera_names')
        return response.get('cameras', []) if response.get('status') == 'ok' else []

    def params_all(self):
        response = self._send_command('params_all')
        return response.get('info', {}) if response.get('status') == 'ok' else {}

    # ===== Gazebo Commands (Simple protocol - complexity on host) =====

    def gazebo_spawn_sdf(self, name, sdf_xml, position, orientation=None):
        """Spawn object from SDF XML (low-level)."""
        orientation = orientation or [0, 0, 0, 1]
        response = self._send_command('gazebo_spawn_sdf',
                                       name=name,
                                       sdf_xml=sdf_xml,
                                       position=position,
                                       orientation=orientation)
        return response.get('status') == 'ok'

    def gazebo_delete(self, name):
        """Delete object from Gazebo."""
        response = self._send_command('gazebo_delete', name=name)
        return response.get('status') == 'ok'

    def gazebo_get_pose(self, name):
        """Get object pose."""
        response = self._send_command('gazebo_get_pose', name=name)
        return response.get('pose') if response.get('status') == 'ok' else None

    def gazebo_set_pose(self, name, position, orientation=None):
        """Set object pose."""
        orientation = orientation or [0, 0, 0, 1]
        response = self._send_command('gazebo_set_pose',
                                       name=name,
                                       position=position,
                                       orientation=orientation)
        return response.get('status') == 'ok'

    def gazebo_is_spawned(self, name):
        """Check if object exists."""
        response = self._send_command('gazebo_is_spawned', name=name)
        return response.get('spawned', False) if response.get('status') == 'ok' else False

    def gazebo_list_models(self):
        """List all models in Gazebo."""
        response = self._send_command('gazebo_list_models')
        return response.get('models', []) if response.get('status') == 'ok' else []

    # ===== Contact Sensor Commands =====

    def contact_get_last_landing(self):
        """Get last detected landing event."""
        response = self._send_command('contact_get_last_landing')
        if response.get('status') == 'ok':
            return response.get('landing')
        return None

    def contact_start_detection(self):
        """Enable contact detection."""
        response = self._send_command('contact_start_detection')
        return response.get('status') == 'ok'

    def contact_stop_detection(self):
        """Disable contact detection."""
        response = self._send_command('contact_stop_detection')
        return response.get('status') == 'ok'

    def contact_clear(self):
        """Clear landing data."""
        response = self._send_command('contact_clear')
        return response.get('status') == 'ok'

    # ===== RGBD Camera Commands =====

    def camera_get_point_cloud(self):
        """Request point cloud capture."""
        response = self._send_command('camera_get_point_cloud')
        return response

    def camera_enable_streaming(self):
        """Enable point cloud streaming in state broadcast."""
        response = self._send_command('camera_enable_streaming')
        return response.get('status') == 'ok'

    def camera_disable_streaming(self):
        """Disable point cloud streaming."""
        response = self._send_command('camera_disable_streaming')
        return response.get('status') == 'ok'

    # ===== System Commands =====

    def get_full_state(self) -> Dict:
        """Get complete system state (robot + gripper)."""
        response = self._send_command('get_full_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    def ping(self) -> float:
        """
        Test connection latency.

        Returns:
            Round-trip time in milliseconds
        """
        start = time.time()
        response = self._send_command('ping')
        latency_ms = (time.time() - start) * 1000

        if response.get('status') == 'ok':
            return latency_ms
        return -1.0

    # ===== State Subscription (Optional) =====

    def subscribe_to_state(self, callback=None):
        """
        Subscribe to state updates at 100Hz.

        Args:
            callback: Function to call with each state update (optional)
                      If None, use get_latest_state() to poll

        Note: This creates a SUB socket and starts receiving state updates
        """
        if self.sub_socket is None:
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(f"tcp://{self.host}:{self.state_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all messages
            print(f"✓ Subscribed to state updates on {self.host}:{self.state_port}")

        if callback:
            # Run callback in blocking loop (user should run in thread)
            while True:
                state = self.sub_socket.recv_json()
                callback(state)

    def get_latest_state(self) -> Optional[Dict]:
        """
        Get latest state from subscription (non-blocking).

        Returns:
            Latest state dict or None if no data available

        Note: Must call subscribe_to_state() first
        """
        if self.sub_socket is None:
            raise RuntimeError("Must call subscribe_to_state() first")

        try:
            # Non-blocking receive
            state = self.sub_socket.recv_json(zmq.NOBLOCK)
            return state
        except zmq.Again:
            return None

    # ===== Cleanup =====

    def close(self):
        """Close connection and cleanup resources."""
        if self.req_socket:
            self.req_socket.close()
        if self.sub_socket:
            self.sub_socket.close()
        if self.context:
            self.context.term()
        print("✓ ZMQ Client closed")


if __name__ == "__main__":
    """Simple test of ZMQ client"""
    import numpy as np

    client = ZMQClient(host='localhost')

    # Test ping
    print("\n[1] Testing connection...")
    latency = client.ping()
    print(f"    Latency: {latency:.2f} ms")

    # Get current state
    print("\n[2] Getting current state...")
    angles = client.get_joint_angles()
    print(f"    Joint angles: {np.round(angles, 3)}")

    # Move robot
    print("\n[3] Moving robot...")
    target = [0.0, 0.3, 0.5, 0.0, 0.0, -0.2, 0.0]
    success = client.move_to_joints(target, timeout=30.0)
    print(f"    Success: {success}")

    # Test gripper
    print("\n[4] Testing gripper...")
    client.gripper_open()
    print("    Gripper opened")

    client.gripper_close()
    print("    Gripper closed")

    # Get full state
    print("\n[5] Getting full state...")
    state = client.get_full_state()
    print(f"    Gripper grasping: {state.get('gripper', {}).get('is_grasping')}")

    client.close()
    print("\n[PASS] ZMQ client test complete")
