#!/usr/bin/env python3
import rospy


class RobotParams:
    """Interface for querying robot configuration parameters."""

    def __init__(self):
        rospy.loginfo("RobotParams: Initialized")

    def get_robot_name(self):
        """
        Get robot class name (e.g., 'sawyer', 'baxter').

        Returns:
            str: Robot name or None
        """
        try:
            return rospy.get_param("/manifest/robot_class", None)
        except Exception as e:
            rospy.logwarn(f"Failed to get robot name: {e}")
            return None

    def get_robot_assemblies(self):
        """
        Get all robot assembly names.

        Returns:
            list: Assembly names (e.g., ['right', 'left', 'torso', 'head'])
        """
        try:
            return rospy.get_param("/robot_config/assembly_names", [])
        except Exception as e:
            rospy.logwarn(f"Failed to get assemblies: {e}")
            return []

    def get_limb_names(self):
        """
        Get articulated limb names (excludes torso, head).

        Returns:
            list: Limb names (e.g., ['right', 'left'])
        """
        assemblies = self.get_robot_assemblies()
        non_limbs = ['torso', 'head']
        return [a for a in assemblies if a not in non_limbs]

    def get_joint_names(self, limb_name):
        """
        Get joint names for specified limb.

        Args:
            limb_name: Name of limb (e.g., 'right')

        Returns:
            list: Joint names from proximal to distal
        """
        try:
            param = f"robot_config/{limb_name}_config/joint_names"
            return rospy.get_param(param, [])
        except Exception as e:
            rospy.logwarn(f"Failed to get joint names for {limb_name}: {e}")
            return []

    def get_camera_names(self):
        """
        Get camera names.

        Returns:
            list: Camera names
        """
        return list(self.get_camera_details().keys())

    def get_camera_details(self):
        """
        Get camera configuration details.

        Returns:
            dict: Camera details
        """
        try:
            return rospy.get_param("/robot_config/camera_config", {})
        except Exception as e:
            rospy.logwarn(f"Failed to get camera details: {e}")
            return {}

    def get_all_info(self):
        """
        Get all robot information.

        Returns:
            dict: Complete robot configuration
        """
        return {
            'robot_name': self.get_robot_name(),
            'assemblies': self.get_robot_assemblies(),
            'limbs': self.get_limb_names(),
            'cameras': self.get_camera_names(),
        }
