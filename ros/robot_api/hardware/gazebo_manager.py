#!/usr/bin/env python3
"""
GazeboManager - Thin wrapper for Gazebo ROS services

Simple, stable interface - just exposes basic Gazebo ROS service calls.
No complex logic - all algorithms handled on host side.
"""

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose
import threading


class GazeboManager:
    """
    Simple wrapper around Gazebo ROS services.

    Exposes only basic operations:
    - spawn_sdf(name, sdf_xml, pose) - Spawn from SDF XML
    - delete(name) - Delete model
    - get_pose(name) - Get model pose
    - set_pose(name, pose) - Set model pose
    - list_models() - List all models
    - is_spawned(name) - Check if model exists
    """

    def __init__(self):
        """Initialize Gazebo ROS service clients."""
        self._lock = threading.Lock()
        self._pose_cache = {}  # Map: model_name -> Pose (from /gazebo/model_states)
        self._twist_cache = {}  # Map: model_name -> Twist (velocities)

        # ROS service clients
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Subscribe to model states for fast pose queries
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb)

        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
            rospy.loginfo("GazeboManager initialized successfully")
        except rospy.ROSException:
            rospy.logwarn("Gazebo spawn service not available")

    def _model_states_cb(self, msg):
        """Cache model poses and twists from /gazebo/model_states topic."""
        with self._lock:
            for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
                self._pose_cache[name] = pose
                self._twist_cache[name] = twist

    def spawn_sdf(self, model_name, sdf_xml, pose):
        """
        Spawn model from SDF XML string.

        Args:
            model_name: Unique name for model instance
            sdf_xml: Complete SDF XML string
            pose: Pose message (geometry_msgs/Pose)

        Returns:
            bool: Success
        """
        try:
            resp = self.spawn_srv(model_name, sdf_xml, "", pose, "world")
            rospy.logdebug(f"Spawned {model_name}")
            return resp.success
        except Exception as e:
            rospy.logerr(f"Failed to spawn {model_name}: {e}")
            return False

    def delete(self, model_name):
        """
        Delete model from Gazebo.

        Args:
            model_name: Name of model to delete

        Returns:
            bool: Success
        """
        try:
            resp = self.delete_srv(model_name)
            with self._lock:
                self._pose_cache.pop(model_name, None)
            rospy.logdebug(f"Deleted {model_name}")
            return resp.success
        except Exception as e:
            rospy.logerr(f"Failed to delete {model_name}: {e}")
            return False

    def get_pose(self, model_name):
        """
        Get model pose (uses cached data for speed).

        Args:
            model_name: Name of model

        Returns:
            dict: {'position': [x,y,z], 'orientation': [x,y,z,w]} or None
        """
        with self._lock:
            if model_name in self._pose_cache:
                pose = self._pose_cache[model_name]
                return {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation': [pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]
                }

        # Fallback to service call if not in cache
        try:
            resp = self.get_state_srv(model_name, "world")
            if resp.success:
                pose = resp.pose
                return {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation': [pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]
                }
        except:
            pass

        return None

    def set_pose(self, model_name, pose):
        """
        Set model pose.

        Args:
            model_name: Name of model
            pose: Pose message (geometry_msgs/Pose)

        Returns:
            bool: Success
        """
        msg = ModelState()
        msg.model_name = model_name
        msg.pose = pose
        msg.reference_frame = "world"

        try:
            resp = self.set_state_srv(msg)
            rospy.logdebug(f"Set pose for {model_name}")
            return resp.success
        except Exception as e:
            rospy.logerr(f"Failed to set pose for {model_name}: {e}")
            return False

    def list_models(self):
        """
        List all models currently in Gazebo.

        Returns:
            list: Model names
        """
        with self._lock:
            return list(self._pose_cache.keys())

    def is_spawned(self, model_name):
        """
        Check if model exists in Gazebo.

        Args:
            model_name: Name to check

        Returns:
            bool: True if model exists
        """
        with self._lock:
            return model_name in self._pose_cache
