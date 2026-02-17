"""
RobotClient - Main client interface for robot control

Simple, clean API for controlling the robot from the host.
Automatically selects the appropriate protocol (ZMQ, HTTP, WebSocket).
"""

from typing import List, Dict, Optional


class RobotClient:
    """
    Main robot client interface.

    Examples:
        # Basic usage with ZMQ (recommended)
        robot = RobotClient(protocol='zmq', host='localhost')
        robot.move_to_joints([0, 0.3, 0.5, 0, 0, 0, 0])
        angles = robot.get_joint_angles()

        # Use HTTP for simple scripts
        robot = RobotClient(protocol='http', host='localhost')
        robot.gripper_open()
    """

    def __init__(self, protocol='zmq', host='localhost', **kwargs):
        """
        Initialize robot client.

        Args:
            protocol: 'zmq', 'http', or 'websocket'
            host: Hostname or IP of container (default: 'localhost')
            **kwargs: Protocol-specific options
        """
        self.protocol = protocol
        self.host = host

        if protocol == 'zmq':
            from .protocols.zmq import ZMQClient
            command_port = kwargs.get('command_port', 5555)
            state_port = kwargs.get('state_port', 5556)
            self._client = ZMQClient(host, command_port, state_port)

        elif protocol == 'http':
            from .protocols.http import HTTPClient
            port = kwargs.get('port', 5000)
            self._client = HTTPClient(host, port)

        elif protocol == 'websocket' or protocol == 'ws':
            from .protocols.websocket import WebSocketClient
            port = kwargs.get('port', 5000)
            self._client = WebSocketClient(host, port)

        else:
            raise ValueError(f"Unknown protocol: {protocol}. Use 'zmq', 'http', or 'websocket'")

    # ===== Robot Control Methods =====

    def move_to_joints(self, angles: List[float], timeout: float = 30.0) -> bool:
        """
        Move robot to target joint positions.

        Args:
            angles: List of 7 joint angles in radians
            timeout: Maximum time to wait (seconds)

        Returns:
            True if successful, False otherwise
        """
        return self._client.move_to_joints(angles, timeout)

    def get_joint_angles(self) -> List[float]:
        """
        Get current joint angles.

        Returns:
            List of 7 joint angles in radians
        """
        return self._client.get_joint_angles()

    def get_endpoint_pose(self) -> Optional[Dict]:
        """
        Get end-effector pose.

        Returns:
            Dict with 'position': [x, y, z], 'orientation': [x, y, z, w]
        """
        return self._client.get_endpoint_pose()

    def get_robot_state(self) -> Dict:
        """
        Get complete robot state.

        Returns:
            Dict with joint_angles, velocities, efforts, endpoint pose, etc.
        """
        return self._client.get_robot_state()

    # ===== Gripper Control Methods =====

    def gripper_open(self) -> bool:
        """Open the gripper fully."""
        return self._client.gripper_open()

    def gripper_close(self) -> bool:
        """Close the gripper (or until grasping object)."""
        return self._client.gripper_close()

    def gripper_set_position(self, position: float) -> bool:
        """
        Set gripper to specific position.

        Args:
            position: Gripper width in meters (0.0 = closed, 0.041667 = open)
        """
        return self._client.gripper_set_position(position)

    def gripper_get_state(self) -> Dict:
        """
        Get gripper state.

        Returns:
            Dict with 'position' and 'is_grasping'
        """
        return self._client.gripper_get_state()

    # ===== Camera Methods =====

    def camera_start(self) -> bool:
        """Start camera streaming."""
        return self._client.camera_start()

    def camera_stop(self) -> bool:
        """Stop camera streaming."""
        return self._client.camera_stop()

    def camera_get_image(self):
        """
        Get current camera image.

        Returns:
            numpy array (H, W, 3) BGR format, or None if unavailable
        """
        return self._client.camera_get_image()

    # ===== Lights Methods =====

    def lights_list(self):
        """List all available lights."""
        return self._client.lights_list()

    def lights_set(self, name, on=True):
        """Set light state (on=True, off=False)."""
        return self._client.lights_set(name, on)

    def lights_get(self, name):
        """Get light state."""
        return self._client.lights_get(name)

    # ===== Head Methods =====

    def head_pan(self):
        """Get current head pan angle in radians."""
        return self._client.head_pan()

    def head_set_pan(self, angle, speed=1.0):
        """Set head pan angle."""
        return self._client.head_set_pan(angle, speed)

    # ===== Head Display Methods =====

    def display_image(self, image):
        """
        Display image on head screen.

        Args:
            image: numpy array (H, W, 3) BGR format
        """
        return self._client.display_image(image)

    def display_clear(self):
        """Clear head display."""
        return self._client.display_clear()

    # ===== Robot Enable Methods =====

    def robot_enable(self):
        """Enable robot (turn on motors)."""
        return self._client.robot_enable()

    def robot_disable(self):
        """Disable robot (turn off motors)."""
        return self._client.robot_disable()

    def robot_reset(self):
        """Reset robot (clear faults)."""
        return self._client.robot_reset()

    def robot_stop(self):
        """Emergency stop robot."""
        return self._client.robot_stop()

    def robot_state(self):
        """Get robot enable state."""
        return self._client.robot_state()

    # ===== Robot Params Methods =====

    def get_robot_name(self):
        """Get robot name (e.g., 'sawyer')."""
        return self._client.params_robot_name()

    def get_limb_names(self):
        """Get limb names."""
        return self._client.params_limb_names()

    def get_joint_names(self, limb='right'):
        """Get joint names for limb."""
        return self._client.params_joint_names(limb)

    def get_camera_names(self):
        """Get camera names."""
        return self._client.params_camera_names()

    def get_robot_info(self):
        """Get all robot configuration info."""
        return self._client.params_all()

    # ===== Gazebo Methods =====

    def gazebo_spawn(self, model_type: str, name: str, position: List[float],
                     orientation: Optional[List[float]] = None) -> bool:
        """
        Spawn an object in Gazebo simulation.

        Args:
            model_type: Model folder name (e.g., 'cube', 'sphere', 'cylinder')
            name: Unique name for this object instance
            position: [x, y, z] position in meters
            orientation: [x, y, z, w] quaternion (default: [0, 0, 0, 1])

        Returns:
            True if successful

        Example:
            robot.gazebo_spawn('cube', 'my_cube', [0.6, 0.0, 0.8])
        """
        return self._client.gazebo_spawn(model_type, name, position, orientation)

    def gazebo_spawn_with_color(self, model_type: str, name: str, position: List[float],
                                color: str, orientation: Optional[List[float]] = None) -> bool:
        """
        Spawn an object with custom color.

        Args:
            model_type: Model folder name
            name: Unique name for this object instance
            position: [x, y, z] position in meters
            color: Gazebo color name (e.g., 'Gazebo/Red', 'Gazebo/Blue', 'Gazebo/Green')
            orientation: [x, y, z, w] quaternion (default: [0, 0, 0, 1])

        Returns:
            True if successful

        Example:
            robot.gazebo_spawn_with_color('sphere', 'red_ball', [0.6, 0.1, 0.8], 'Gazebo/Red')
        """
        return self._client.gazebo_spawn_with_color(model_type, name, position, color, orientation)

    def gazebo_despawn(self, name: str) -> bool:
        """
        Remove an object from Gazebo.

        Args:
            name: Object name to remove

        Returns:
            True if successful
        """
        return self._client.gazebo_despawn(name)

    def gazebo_despawn_all(self) -> int:
        """
        Remove all tracked objects from Gazebo.

        Returns:
            Number of objects removed
        """
        return self._client.gazebo_despawn_all()

    def gazebo_get_pose(self, name: str) -> Optional[Dict]:
        """
        Get object pose in Gazebo.

        Args:
            name: Object name

        Returns:
            Dict with 'position': [x, y, z], 'orientation': [x, y, z, w], or None if not found

        Example:
            pose = robot.gazebo_get_pose('my_cube')
            if pose:
                print(f"Position: {pose['position']}")
        """
        return self._client.gazebo_get_pose(name)

    def gazebo_set_pose(self, name: str, position: List[float],
                       orientation: Optional[List[float]] = None) -> bool:
        """
        Set object pose in Gazebo.

        Args:
            name: Object name
            position: [x, y, z] position
            orientation: [x, y, z, w] quaternion (default: [0, 0, 0, 1])

        Returns:
            True if successful
        """
        return self._client.gazebo_set_pose(name, position, orientation)

    def gazebo_is_spawned(self, name: str) -> bool:
        """
        Check if object exists in Gazebo.

        Args:
            name: Object name

        Returns:
            True if object exists
        """
        return self._client.gazebo_is_spawned(name)

    def gazebo_list_objects(self) -> List[str]:
        """
        List all spawned object names.

        Returns:
            List of object names
        """
        return self._client.gazebo_list_objects()

    def gazebo_available_models(self) -> List[str]:
        """
        Get list of available object models.

        Returns:
            List of model folder names (e.g., ['cube', 'sphere', 'cylinder', ...])
        """
        return self._client.gazebo_available_models()

    def gazebo_spawn_randomized(self, model_types: List[str], area_x: List[float],
                                area_y: List[float], z_height: float,
                                min_dist: float = 0.05, instances_per_type: int = 1) -> List[str]:
        """
        Spawn multiple objects randomly with collision avoidance.

        Args:
            model_types: List of object types (e.g., ['cube', 'sphere', 'cylinder'])
            area_x: [x_min, x_max] workspace bounds in meters
            area_y: [y_min, y_max] workspace bounds in meters
            z_height: Spawn height in meters
            min_dist: Minimum separation between objects in meters
            instances_per_type: Number of instances per object type

        Returns:
            List of spawned object names

        Example:
            objects = robot.gazebo_spawn_randomized(
                model_types=['cube', 'sphere', 'cylinder'],
                area_x=[0.5, 0.8],
                area_y=[-0.3, 0.3],
                z_height=0.8,
                min_dist=0.08,
                instances_per_type=2
            )
            print(f"Spawned: {objects}")
        """
        return self._client.gazebo_spawn_randomized(
            model_types, area_x, area_y, z_height, min_dist, instances_per_type
        )

    # ===== System Methods =====

    def get_full_state(self) -> Dict:
        """
        Get complete system state (robot + gripper).

        Returns:
            Dict with robot and gripper states
        """
        return self._client.get_full_state()

    def ping(self) -> float:
        """
        Test connection latency.

        Returns:
            Round-trip time in milliseconds
        """
        return self._client.ping()

    def close(self):
        """Close connection and cleanup resources."""
        if hasattr(self._client, 'close'):
            self._client.close()

    def __enter__(self):
        """Support context manager (with statement)."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup when exiting context manager."""
        self.close()
