"""
GazeboClient - Gazebo simulation control with full object management logic

All complexity handled on host side:
- Model discovery from filesystem
- SDF reading and color modification
- Collision detection
- Randomized spawning algorithms
- Object tracking

Container just exposes basic ROS service calls.
"""

import os
import re
import numpy as np
from typing import List, Dict, Optional
from scipy.spatial.transform import Rotation as R
from .base_client import BaseClient


class GazeboClient(BaseClient):
    """
    Gazebo simulation control client (with full logic on host).

    Handles all complexity:
    - Discovers models from GAZEBO_MODEL_PATH or custom paths
    - Reads and modifies SDF files
    - Collision detection for randomized spawning
    - Object tracking

    Container just executes basic Gazebo ROS service calls.
    """

    def __init__(self, protocol='zmq', host='localhost', model_paths=None, **kwargs):
        """
        Initialize Gazebo client.

        Args:
            protocol: Communication protocol ('zmq', 'http', 'websocket')
            host: Container hostname
            model_paths: List of paths to search for models (default: from GAZEBO_MODEL_PATH env)
            **kwargs: Protocol-specific options
        """
        super().__init__(protocol, host, **kwargs)

        # Track spawned objects (host-side tracking)
        self._spawned = {}  # name -> {'model_type': str, 'position': [...], ...}

        # Model paths for discovery
        self.model_paths = model_paths or self._get_model_paths_from_env()

    def _get_model_paths_from_env(self):
        """Get model paths from GAZEBO_MODEL_PATH environment variable."""
        paths_str = os.environ.get('GAZEBO_MODEL_PATH', '')
        if not paths_str:
            # Default fallback
            return ['/usr/share/gazebo/models']

        return [p.strip() for p in paths_str.split(':') if p.strip()]

    def _find_sdf_file(self, model_type):
        """
        Find SDF file for given model type.

        Args:
            model_type: Model folder name (e.g., 'cube', 'sphere')

        Returns:
            str: Path to model.sdf or None if not found
        """
        for base_path in self.model_paths:
            model_dir = os.path.join(base_path, model_type)
            if not os.path.exists(model_dir):
                continue

            # Try model.sdf first, then model.urdf
            for filename in ['model.sdf', 'model.urdf']:
                sdf_path = os.path.join(model_dir, filename)
                if os.path.exists(sdf_path):
                    return sdf_path

        return None

    def _read_sdf(self, sdf_path):
        """Read SDF XML from file."""
        with open(sdf_path, 'r') as f:
            return f.read()

    def _modify_sdf_color(self, sdf_xml, color):
        """
        Modify SDF XML to change material color.

        Args:
            sdf_xml: Original SDF XML string
            color: Gazebo color name (e.g., 'Gazebo/Red')

        Returns:
            str: Modified SDF XML
        """
        # Replace all material/script/name tags
        pattern = r'<material><script><name>[^<]+</name></script></material>'
        replacement = f'<material><script><name>{color}</name></script></material>'
        return re.sub(pattern, replacement, sdf_xml)

    def spawn(self, model_type: str, name: str, position: List[float],
              orientation: Optional[List[float]] = None) -> bool:
        """
        Spawn an object in Gazebo.

        Args:
            model_type: Model folder name (e.g., 'cube', 'sphere')
            name: Unique name for this instance
            position: [x, y, z] position in meters
            orientation: [x, y, z, w] quaternion (default: [0, 0, 0, 1])

        Returns:
            bool: Success
        """
        orientation = orientation or [0, 0, 0, 1]

        # Find and read SDF file (host-side logic)
        sdf_path = self._find_sdf_file(model_type)
        if not sdf_path:
            print(f"Error: Model '{model_type}' not found in paths: {self.model_paths}")
            return False

        sdf_xml = self._read_sdf(sdf_path)

        # Send SDF to container for spawning
        success = self._client.gazebo_spawn_sdf(name, sdf_xml, position, orientation)

        if success:
            # Track spawned object (host-side)
            self._spawned[name] = {
                'model_type': model_type,
                'position': position.copy(),
                'orientation': orientation.copy()
            }

        return success

    def spawn_with_color(self, model_type: str, name: str, position: List[float],
                         color: str, orientation: Optional[List[float]] = None) -> bool:
        """
        Spawn an object with custom color.

        Args:
            model_type: Model folder name
            name: Unique name for this instance
            position: [x, y, z] position in meters
            color: Gazebo color name (e.g., 'Gazebo/Red', 'Gazebo/Blue', 'Gazebo/Green')
            orientation: [x, y, z, w] quaternion (default: [0, 0, 0, 1])

        Returns:
            bool: Success
        """
        orientation = orientation or [0, 0, 0, 1]

        # Find and read SDF (host-side)
        sdf_path = self._find_sdf_file(model_type)
        if not sdf_path:
            print(f"Error: Model '{model_type}' not found")
            return False

        sdf_xml = self._read_sdf(sdf_path)

        # Modify color (host-side logic)
        sdf_xml = self._modify_sdf_color(sdf_xml, color)

        # Send modified SDF to container
        success = self._client.gazebo_spawn_sdf(name, sdf_xml, position, orientation)

        if success:
            self._spawned[name] = {
                'model_type': model_type,
                'position': position.copy(),
                'orientation': orientation.copy(),
                'color': color
            }

        return success

    def despawn(self, name: str) -> bool:
        """Remove an object from Gazebo."""
        success = self._client.gazebo_delete(name)
        if success:
            self._spawned.pop(name, None)
        return success

    def despawn_all(self) -> int:
        """Remove all tracked objects. Returns count."""
        count = 0
        for name in list(self._spawned.keys()):
            if self.despawn(name):
                count += 1
        return count

    def get_pose(self, name: str) -> Optional[Dict]:
        """Get object pose from Gazebo."""
        return self._client.gazebo_get_pose(name)

    def set_pose(self, name: str, position: List[float],
                 orientation: Optional[List[float]] = None) -> bool:
        """Set object pose."""
        orientation = orientation or [0, 0, 0, 1]
        return self._client.gazebo_set_pose(name, position, orientation)

    def is_spawned(self, name: str) -> bool:
        """Check if object exists in Gazebo."""
        return self._client.gazebo_is_spawned(name)

    def list_objects(self) -> List[str]:
        """List all spawned objects (tracked on host)."""
        return list(self._spawned.keys())

    def available_models(self) -> List[str]:
        """
        Get list of available models (scanned from filesystem).

        Returns:
            List of model folder names
        """
        models = []
        for base_path in self.model_paths:
            if not os.path.exists(base_path):
                continue

            try:
                for folder in os.listdir(base_path):
                    folder_path = os.path.join(base_path, folder)
                    if not os.path.isdir(folder_path):
                        continue

                    # Check if has model.sdf or model.urdf
                    if (os.path.exists(os.path.join(folder_path, 'model.sdf')) or
                        os.path.exists(os.path.join(folder_path, 'model.urdf'))):
                        if folder not in models:
                            models.append(folder)
            except (OSError, PermissionError):
                continue

        return sorted(models)

    def spawn_randomized(self, model_types: List[str], area_x: List[float],
                        area_y: List[float], z_height: float,
                        min_dist: float = 0.05, instances_per_type: int = 1) -> List[str]:
        """
        Spawn multiple objects randomly with collision avoidance (HOST-SIDE ALGORITHM).

        Args:
            model_types: List of object types (e.g., ['cube', 'sphere'])
            area_x: [x_min, x_max] workspace bounds
            area_y: [y_min, y_max] workspace bounds
            z_height: Spawn height
            min_dist: Minimum separation between objects
            instances_per_type: Objects per type

        Returns:
            List of spawned object names
        """
        # Color palette
        colors = [
            "Gazebo/Red", "Gazebo/Blue", "Gazebo/Green", "Gazebo/Orange",
            "Gazebo/Yellow", "Gazebo/Purple", "Gazebo/Turquoise", "Gazebo/White"
        ]

        spawned_objects = []
        spawned_positions = []  # Track positions for collision detection

        # Spawn objects with collision avoidance (HOST-SIDE LOGIC)
        color_idx = 0
        for model_type in model_types:
            for instance in range(instances_per_type):
                name = f"{model_type}_{instance}"

                # Try to find valid position (collision avoidance)
                attempts = 0
                max_attempts = 100
                valid_position = None

                while attempts < max_attempts:
                    # Random position in area
                    rx = np.random.uniform(area_x[0], area_x[1])
                    ry = np.random.uniform(area_y[0], area_y[1])

                    # Check collision with existing objects
                    collision = False
                    for pos in spawned_positions:
                        dist = np.sqrt((rx - pos[0])**2 + (ry - pos[1])**2)
                        if dist < min_dist:
                            collision = True
                            break

                    if not collision:
                        valid_position = [rx, ry, z_height]
                        break

                    attempts += 1

                if not valid_position:
                    print(f"Warning: Could not find valid position for {name} after {max_attempts} attempts")
                    continue

                # Random orientation
                angle = np.random.uniform(0, 2 * np.pi)
                quat = R.from_euler('XYZ', [0, 0, angle]).as_quat()
                orientation = quat.tolist()

                # Spawn with color
                color = colors[color_idx % len(colors)]
                success = self.spawn_with_color(model_type, name, valid_position, color, orientation)

                if success:
                    spawned_objects.append(name)
                    spawned_positions.append(valid_position)
                    color_idx += 1

        return spawned_objects
