#!/usr/bin/env python
import os
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point

class GazeboObjectManager:
    def __init__(self):
        rospy.init_node("spawn_objects", anonymous=True)

        # Absolute path to models folder
        self.model_root = rospkg.RosPack().get_path('simulation') + "/models"

        # Ensure Gazebo can find models
        self.add_model_path(self.model_root)

        # Wait for spawn service
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rospy.loginfo("Gazebo Object Manager initialized.")

    def add_model_path(self, path):
        """Add model path to GAZEBO_MODEL_PATH if not present."""
        abs_path = os.path.abspath(path)
        current = os.environ.get("GAZEBO_MODEL_PATH", "")
        paths = current.split(":") if current else []

        if abs_path not in paths:
            os.environ["GAZEBO_MODEL_PATH"] = current + ":" + abs_path if current else abs_path
            rospy.loginfo("Added '{}' to GAZEBO_MODEL_PATH".format(abs_path))

    def spawn_model(self, folder_name, model_name=None, pose=None, relative_to='world'):
        """Spawn a model from a folder in the models directory."""
        folder_path = os.path.join(self.model_root, folder_name)
        if not os.path.isdir(folder_path):
            rospy.logerr("Model folder '{}' does not exist.".format(folder_name))
            return

        # Look for any .sdf or .urdf file in the folder
        sdf_files = [f for f in os.listdir(folder_path) if f.endswith(".sdf")]
        urdf_files = [f for f in os.listdir(folder_path) if f.endswith(".urdf")]

        if sdf_files:
            file_path = os.path.join(folder_path, sdf_files[0])
            sdf_type = True
        elif urdf_files:
            file_path = os.path.join(folder_path, urdf_files[0])
            sdf_type = False
        else:
            rospy.logerr("No .sdf or .urdf file found in folder '{}'".format(folder_name))
            return

        model_name = model_name or folder_name
        pose = pose or Pose(position=Point(0, 0, 0.5))

        with open(file_path, "r") as f:
            xml = f.read()

        try:
            if sdf_type:
                self.spawn_sdf(model_name, xml, "", pose, relative_to)
            else:
                rospy.logwarn("URDF spawning not implemented in this snippet.")
            rospy.loginfo("Spawned model '{}' from folder '{}'".format(model_name, folder_name))
        except rospy.ServiceException as e:
            rospy.logerr("Failed to spawn model '{}': {}".format(model_name, e))

    def spawn_table(self, name='table', pose=None):
        """Convenience method to spawn a table from a known folder."""
        table_pose = pose or Pose(position=Point(x=0.75, y=0, z=0))
        self.spawn_model("cafe_table", model_name=name, pose=table_pose)

if __name__ == "__main__":
    manager = GazeboObjectManager()

    # Example usage
    manager.spawn_table(name='cafe_table')
    manager.spawn_table(name='table2', pose=Pose(position=Point(x=1.65, y=0, z=0)))
    manager.spawn_model("030_fork", model_name="fork_1", pose=Pose(position=Point(0.5, 0, 1.5)))
    manager.spawn_model("030_fork", model_name="fork_2", pose=Pose(position=Point(0.6, 0, 1.5)))
    manager.spawn_model("030_fork", model_name="fork_3", pose=Pose(position=Point(0.7, 0, 1.5)))
    manager.spawn_model("030_fork", model_name="fork_4", pose=Pose(position=Point(0.8, 0, 1.5)))
    manager.spawn_model("030_fork", model_name="fork_5", pose=Pose(position=Point(0.9, 0, 1.5)))

    rospy.spin()
