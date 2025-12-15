import torch
import numpy as np
import torchvision.transforms.functional as TF

class RotationTransform:
    @staticmethod
    def to_gripper_frame(tensor, angle_deg):

        return TF.rotate(tensor, -angle_deg)

    @staticmethod
    def to_world_frame(tensor, angle_deg):

        return TF.rotate(tensor, angle_deg)

    @staticmethod
    def rotate_pixel(u, v, angle_deg, h, w, to_gripper_frame=True):

        # Determine direction
        eff_angle = -angle_deg if to_gripper_frame else angle_deg
        angle_rad = np.deg2rad(eff_angle)
        
        # Center
        cy, cx = h / 2.0, w / 2.0
        
        # Shift to center
        y = u - cy
        x = v - cx
        

        x_new = x * np.cos(angle_rad) - y * np.sin(angle_rad)
        y_new = x * np.sin(angle_rad) + y * np.cos(angle_rad)
        
        # Shift back
        v_new = int(x_new + cx)
        u_new = int(y_new + cy)
        
        # Clamp to be safe
        v_new = min(max(v_new, 0), w - 1)
        u_new = min(max(u_new, 0), h - 1)
        
        return u_new, v_new