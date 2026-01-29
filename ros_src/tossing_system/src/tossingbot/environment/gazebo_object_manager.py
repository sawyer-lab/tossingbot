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

class GazeboObjectManager:
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
            default_roots.append(os.path.join(self.rospack.get_path('environments'), 'models'))
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
        try:
            self.delete_srv(model_name)
            with self._lock:
                self.spawned.pop(model_name, None)
                self._pose_cache.pop(model_name, None)
            return True
        except: return False

    def despawn_all(self):
        names = list(self.spawned.keys())
        for name in names:
            self.despawn(name)

    def set_pose(self, model_name, pose):
        msg = ModelState()
        msg.model_name = model_name
        msg.pose = pose
        msg.reference_frame = "world"
        try:
            self.set_state_srv(msg)
            return True
        except: return False

    def is_spawned(self, model_name):
        with self._lock:
            return model_name in self._pose_cache

    def spawn_randomized(self, model_folders, area_x, area_y, z_height=0.85, min_dist=0.12):
        generated = {} 
        for folder in model_folders:
            base_name = folder
            name = base_name
            counter = 1
            while name in generated or self.is_spawned(name):
                name = f"{base_name}_{counter}"
                counter += 1
            valid = False
            attempts = 0
            while not valid and attempts < 20:
                rx, ry = np.random.uniform(area_x[0], area_x[1]), np.random.uniform(area_y[0], area_y[1])
                collision = False
                for other_data in generated.values():
                    other_pose = other_data['pose']
                    dist = np.sqrt((rx - other_pose.position.x)**2 + (ry - other_pose.position.y)**2)
                    if dist < min_dist:
                        collision = True; break
                if not collision:
                    valid = True
                    # q = tft.quaternion_from_euler(0, 0, np.random.uniform(0, 2*np.pi))
                    q = R.from_euler('XYZ', [0, 0, np.random.uniform(0, 2*np.pi)]).as_quat()
                    pose = Pose(position=Point(rx, ry, z_height), orientation=Quaternion(*q))
                    generated[name] = {'pose': pose, 'folder': folder}
                    with self._lock: self.spawned[name] = {'folder': folder} 
                attempts += 1
        for name, data in generated.items():
            self.spawn(data['folder'], model_name=name, pose=data['pose'])
        return generated

    def _find_sdf(self, folder_name):
        paths = os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
        for path in paths:
            target_dir = os.path.join(path, folder_name)
            if os.path.isdir(target_dir):
                for f in ["model.sdf", "model.urdf"]:
                    if os.path.exists(os.path.join(target_dir, f)):
                        return os.path.join(target_dir, f)
        return None
