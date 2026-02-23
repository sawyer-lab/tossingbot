#!/usr/bin/env python
import rospy
import rospkg
import os
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates


class GazeboObjectClient:
    """
    Low-level client for direct Gazebo ROS service calls.
    
    Handles:
    - ROS service proxies and subscriber setup
    - Basic spawn/despawn operations
    - Pose getting/setting
    - Model existence checks
    - SDF file discovery
    """
    def __init__(self, model_roots=None, wait_for_services=True):
        if rospy.get_node_uri() is None:
            rospy.init_node("gazebo_object_manager", anonymous=True)
            
        self._lock = threading.Lock()
        self.spawned = {}  # Map: model_name -> metadata
        self._pose_cache = {} # Map: model_name -> Pose
        
        self.rospack = rospkg.RosPack()
        default_roots = []
        try:
            default_roots.append(os.path.join(self.rospack.get_path('simulation'), 'models'))
            default_roots.append(os.path.join(self.rospack.get_path('tossingbot_environments'), 'models'))
        except rospkg.ResourceNotFound:
            pass
            
        roots = (model_roots or []) + default_roots
        for r in roots:
            self.add_model_path(r)

        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb)

        if wait_for_services:
            try:
                rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
            except rospy.ROSException:
                rospy.logwarn("Gazebo spawn service not available.")

    def add_model_path(self, path):
        abs_path = os.path.abspath(path)
        if not os.path.exists(abs_path): return
        current = os.environ.get("GAZEBO_MODEL_PATH", "")
        if abs_path not in current.split(":"):
            os.environ["GAZEBO_MODEL_PATH"] = f"{current}:{abs_path}" if current else abs_path

    def _model_states_cb(self, msg):
        with self._lock:
            for name, pose in zip(msg.name, msg.pose):
                self._pose_cache[name] = pose

    def get_pose(self, model_name):
        with self._lock:
            if model_name in self._pose_cache:
                return self._pose_cache[model_name]
        try:
            resp = self.get_state_srv(model_name, "world")
            if resp.success: return resp.pose
        except: pass
        return None

    def spawn(self, model_folder, model_name=None, pose=None):
        name = model_name or model_folder
        if self.is_spawned(name): return name
        sdf_path = self._find_sdf(model_folder)
        if not sdf_path: return None
        with open(sdf_path, 'r') as f:
            xml = f.read()
        initial_pose = pose or Pose(position=Point(0, 0, 0.5), orientation=Quaternion(0,0,0,1))
        try:
            self.spawn_srv(name, xml, "", initial_pose, "world")
            with self._lock:
                self.spawned[name] = {'folder': model_folder, 'sdf': sdf_path}
            return name
        except: return None

    def despawn(self, model_name):
        if not self.is_spawned(model_name):
            rospy.logwarn(f"Cannot despawn {model_name}: not in Gazebo")
            return False
        try:
            resp = self.delete_srv(model_name)
            with self._lock:
                self.spawned.pop(model_name, None)
                self._pose_cache.pop(model_name, None)
            rospy.logdebug(f"Successfully despawned {model_name}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service exception while despawning {model_name}: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error despawning {model_name}: {e}")
            return False

    def set_pose(self, model_name, pose):
        if not self.is_spawned(model_name):
            rospy.logerr(f"Cannot set pose for {model_name}: model does not exist in Gazebo")
            return False
        msg = ModelState()
        msg.model_name = model_name
        msg.pose = pose
        msg.reference_frame = "world"
        try:
            resp = self.set_state_srv(msg)
            if resp.success:
                rospy.logdebug(f"Successfully set pose for {model_name}")
                return True
            else:
                rospy.logwarn(f"SetModelState returned success=False for {model_name}: {resp.status_message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service exception while setting pose for {model_name}: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error setting pose for {model_name}: {e}")
            return False

    def is_spawned(self, model_name):
        """Check if model exists in Gazebo pose cache (updated by subscriber)."""
        with self._lock:
            # Check pose cache which is continuously updated by /gazebo/model_states
            in_cache = model_name in self._pose_cache
            in_spawned = model_name in self.spawned
            
            # If tracking mismatch, sync them
            if in_cache and not in_spawned:
                rospy.logdebug(f"{model_name} found in Gazebo but not in tracking, syncing")
                self.spawned[model_name] = {'folder': 'unknown'}
            elif not in_cache and in_spawned:
                rospy.logdebug(f"{model_name} in tracking but not in Gazebo, removing from tracking")
                self.spawned.pop(model_name, None)
                return False
            
            return in_cache

    def _find_sdf(self, folder_name):
        paths = os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
        for path in paths:
            target_dir = os.path.join(path, folder_name)
            if os.path.isdir(target_dir):
                for f in ["model.sdf", "model.urdf"]:
                    if os.path.exists(os.path.join(target_dir, f)):
                        return os.path.join(target_dir, f)
        return None


class GazeboObjectManager:
    """
    High-level object manager with advanced spawning logic.
    
    Handles:
    - Color customization via SDF manipulation
    - Randomized multi-object spawning with collision detection
    - Batch operations
    """
    
    def __init__(self, client=None, model_roots=None, wait_for_services=True):
        """
        Initialize high-level manager.
        
        Args:
            client: Optional GazeboObjectClient instance (creates one if None)
            model_roots: Additional model paths to add
            wait_for_services: Whether to wait for Gazebo services
        """
        self.client = client or GazeboObjectClient(model_roots, wait_for_services)
    
    def spawn_with_color(self, model_folder, model_name=None, pose=None, color="Gazebo/Red"):
        """
        Spawn a model with a specific color by modifying the SDF.
        
        Args:
            model_folder: Folder name of the model
            model_name: Optional custom name for this instance
            pose: Initial pose
            color: Gazebo material name (e.g., "Gazebo/Red", "Gazebo/Blue")
        """
        name = model_name or model_folder
        if self.client.is_spawned(name):
            return name
        
        sdf_path = self.client._find_sdf(model_folder)
        if not sdf_path:
            return None
        
        # Read and modify SDF to change color
        with open(sdf_path, 'r') as f:
            xml = f.read()
        
        # Replace all material/script/name tags with the desired color
        import re
        xml = re.sub(
            r'<material><script><name>[^<]+</name></script></material>',
            f'<material><script><name>{color}</name></script></material>',
            xml
        )
        
        initial_pose = pose or Pose(position=Point(0, 0, 0.5), orientation=Quaternion(0,0,0,1))
        
        try:
            self.client.spawn_srv(name, xml, "", initial_pose, "world")
            with self.client._lock:
                self.client.spawned[name] = {'folder': model_folder, 'sdf': sdf_path, 'color': color}
            rospy.logdebug(f"Spawned {name} with color {color}")
            return name
        except Exception as e:
            rospy.logerr(f"Failed to spawn {name}: {e}")
            return None

    def despawn_all(self):
        """Despawn all tracked objects with robust error handling."""
        names = list(self.client.spawned.keys())
        if not names:
            rospy.logdebug("No objects to despawn")
            return
        
        rospy.loginfo(f"Despawning {len(names)} objects...")
        success_count = 0
        
        for name in names:
            try:
                if self.client.despawn(name):
                    success_count += 1
            except Exception as e:
                rospy.logerr(f"Exception despawning {name}: {e}")
                # Force remove from tracking even if despawn failed
                with self.client._lock:
                    self.client.spawned.pop(name, None)
                    self.client._pose_cache.pop(name, None)
        
        # Ensure tracking dictionaries are completely cleared
        with self.client._lock:
            self.client.spawned.clear()
            self.client._pose_cache.clear()
        
        rospy.loginfo(f"Despawned {success_count}/{len(names)} objects, tracking cleared")
        # Give Gazebo time to process deletions
        rospy.sleep(0.5)

    # Pass-through methods for backward compatibility
    def spawn(self, model_folder, model_name=None, pose=None):
        """Spawn object (delegates to low-level client)."""
        return self.client.spawn(model_folder, model_name, pose)
    
    def despawn(self, model_name):
        """Despawn object (delegates to low-level client)."""
        return self.client.despawn(model_name)
    
    def get_pose(self, model_name):
        """Get object pose (delegates to low-level client)."""
        return self.client.get_pose(model_name)
    
    def set_pose(self, model_name, pose):
        """Set object pose (delegates to low-level client)."""
        return self.client.set_pose(model_name, pose)
    
    def is_spawned(self, model_name):
        """Check if object exists (delegates to low-level client)."""
        return self.client.is_spawned(model_name)

    def spawn_randomized(self, model_folders, area_x, area_y, z_height=0.85, min_dist=0.12, instances_per_type=None):
        """
        Spawn multiple instances of each object type.
        If instances_per_type is None, spawns as many as fit in the workspace.
        
        Args:
            model_folders: List of object types to spawn
            area_x, area_y: Spawning area bounds
            z_height: Height to spawn objects at
            min_dist: Minimum distance between objects
            instances_per_type: Number of instances per type, or None for "fill workspace"
        """
        # Color palette for visual distinction between instances
        colors = [
            "Gazebo/Red", "Gazebo/Blue", "Gazebo/Green", "Gazebo/Orange",
            "Gazebo/Yellow", "Gazebo/Purple", "Gazebo/Turquoise", "Gazebo/White"
        ]
        
        # Shuffle colors for variety each scene
        import random
        shuffled_colors = colors.copy()
        random.shuffle(shuffled_colors)
        
        generated = {}
        total_attempts = 0
        global_color_idx = 0
        
        # If instances_per_type is None, fill workspace mode
        if instances_per_type is None:
            # Try to spawn objects until we can't fit anymore
            max_total_attempts = 500  # Safety limit
            consecutive_failures = 0
            max_consecutive_failures = len(model_folders) * 10  # Give up after this many failures
            
            instance_counts = {folder: 0 for folder in model_folders}
            
            while consecutive_failures < max_consecutive_failures and total_attempts < max_total_attempts:
                # Cycle through object types
                folder = model_folders[len(generated) % len(model_folders)]
                base_name = folder
                instance_idx = instance_counts[folder]
                name = f"{base_name}_{instance_idx}"
                
                # Skip if already spawned
                if name in generated or self.client.is_spawned(name):
                    consecutive_failures += 1
                    continue
                
                # Try to find valid position
                valid = False
                attempts = 0
                while not valid and attempts < 30:
                    total_attempts += 1
                    rx, ry = np.random.uniform(area_x[0], area_x[1]), np.random.uniform(area_y[0], area_y[1])
                    collision = False
                    
                    for other_data in generated.values():
                        other_pose = other_data['pose']
                        dist = np.sqrt((rx - other_pose.position.x)**2 + (ry - other_pose.position.y)**2)
                        if dist < min_dist:
                            collision = True
                            break
                    
                    if not collision:
                        valid = True
                        q = R.from_euler('XYZ', [0, 0, np.random.uniform(0, 2*np.pi)]).as_quat()
                        pose = Pose(position=Point(rx, ry, z_height), orientation=Quaternion(*q))
                        
                        color = shuffled_colors[global_color_idx % len(shuffled_colors)]
                        global_color_idx += 1
                        
                        generated[name] = {
                            'pose': pose,
                            'folder': folder,
                            'color': color,
                            'instance': instance_idx
                        }
                        with self.client._lock:
                            self.client.spawned[name] = {'folder': folder}
                        
                        instance_counts[folder] += 1
                        consecutive_failures = 0  # Reset on success
                    
                    attempts += 1
                
                if not valid:
                    consecutive_failures += 1
            
            rospy.loginfo(f"Fill mode: Spawned {len(generated)} objects after {total_attempts} attempts")
            for folder, count in instance_counts.items():
                rospy.loginfo(f"  {folder}: {count} instances")
        
        else:
            # Fixed instances per type mode
            max_total_attempts = len(model_folders) * instances_per_type * 30
            
            for folder in model_folders:
                base_name = folder
                
                for instance_idx in range(instances_per_type):
                    if total_attempts >= max_total_attempts:
                        rospy.logwarn(f"Reached maximum spawn attempts ({max_total_attempts}), stopping")
                        break
                    
                    name = f"{base_name}_{instance_idx}" if instances_per_type > 1 else base_name
                    
                    if name in generated or self.client.is_spawned(name):
                        rospy.logdebug(f"Skipping {name} - already spawned")
                        continue
                    
                    valid = False
                    attempts = 0
                    while not valid and attempts < 30:
                        total_attempts += 1
                        rx, ry = np.random.uniform(area_x[0], area_x[1]), np.random.uniform(area_y[0], area_y[1])
                        collision = False
                        
                        for other_data in generated.values():
                            other_pose = other_data['pose']
                            dist = np.sqrt((rx - other_pose.position.x)**2 + (ry - other_pose.position.y)**2)
                            if dist < min_dist:
                                collision = True
                                break
                        
                        if not collision:
                            valid = True
                            q = R.from_euler('XYZ', [0, 0, np.random.uniform(0, 2*np.pi)]).as_quat()
                            pose = Pose(position=Point(rx, ry, z_height), orientation=Quaternion(*q))
                            
                            color = shuffled_colors[global_color_idx % len(shuffled_colors)]
                            global_color_idx += 1
                            
                            generated[name] = {
                                'pose': pose,
                            'folder': folder,
                            'color': color,
                            'instance': instance_idx
                        }
                        with self.client._lock:
                            self.client.spawned[name] = {'folder': folder}
                    
                    attempts += 1
                
                if not valid:
                    rospy.logwarn(f"Could not find valid position for {name} after {attempts} attempts")
        
        # Spawn all generated objects
        for name, data in generated.items():
            self.spawn_with_color(data['folder'], model_name=name, pose=data['pose'], color=data['color'])
        
        rospy.loginfo(f"Spawned {len(generated)} objects: {list(generated.keys())}")
        return generated
