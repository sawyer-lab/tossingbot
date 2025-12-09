import numpy as np
from geometry_msgs.msg import Quaternion

class RotationPrimitive:
    def __init__(self, num_rotations=4):
        self.num_rotations = num_rotations
        self.angle_step = 180.0 / num_rotations

    def get_angle(self, idx):
        return (idx % self.num_rotations) * self.angle_step

    def get_quaternion(self, idx):
        # Get base angle (0, 45, 90...)
        angle_deg = self.get_angle(idx)
        
        # --- THE OFFSET FIX ---
        # If your gripper fingers are aligned with X, but 0-deg means aligned with Y,
        # we add 90 degrees here.
        # TRY THIS: Add +90 if your grasps are consistently perpendicular to the banana.
        corrected_angle = angle_deg 
        
        yaw = np.deg2rad(corrected_angle)
        
        # Fixed Down Orientation params (Roll=180)
        roll = np.pi 
        pitch = 0.0
        
        cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5); sr = np.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    @staticmethod
    def get_random_flat_quaternion():
        """Random Yaw for object reset."""
        yaw = np.random.uniform(0, 2 * np.pi)
        return [0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]