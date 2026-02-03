#!/usr/bin/env python
import os
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

class GazeboObjectManager:
    def __init__(self, wait_for_services=True):
        # Only init node if not already initialized
        if rospy.get_node_uri() is None:
            rospy.init_node("gazebo_object_manager", anonymous=True)

        # Model Paths
        self.rospack = rospkg.RosPack()
        self.paths = []
        try:
            self.paths.append(self.rospack.get_path('simulation') + "/models")
        except: pass
        try:
            self.paths.append(self.rospack.get_path('environments') + "/models")
        except: pass
        
        # Service Proxies
        self.spawn_sdf_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        if wait_for_services:
            rospy.loginfo("Waiting for Gazebo services...")
            self.spawn_sdf_srv.wait_for_service()
            self.delete_model_srv.wait_for_service()
            rospy.loginfo("Gazebo Object Manager connected.")

    def _find_model_file(self, model_name):
        """Search for model.sdf in known paths."""
        for root in self.paths:
            candidate = os.path.join(root, model_name, "model.sdf")
            if os.path.exists(candidate):
                return candidate
        return None

    def spawn(self, model_folder, model_name, pose):
        """
        Generic spawn method. 
        Args:
            model_folder: Name of folder in models/ directory (e.g. 'cube')
            model_name: Name in Gazebo (e.g. 'toss_cube')
            pose: geometry_msgs/Pose
        """
        sdf_path = self._find_model_file(model_folder)
        if not sdf_path:
            rospy.logerr(f"Model '{model_folder}' not found in {self.paths}")
            return False

        with open(sdf_path, "r") as f:
            xml = f.read()

        try:
            self.spawn_sdf_srv(model_name, xml, "", pose, "world")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")
            return False

    def spawn_sdf_model(self, model_folder, model_name, pose):
        """Alias for spawn, compatible with test script usage."""
        return self.spawn(model_folder, model_name, pose)

    def despawn(self, model_name):
        try:
            self.delete_model_srv(model_name)
            return True
        except rospy.ServiceException as e:
            # Common error if model doesn't exist, just ignore
            return False
