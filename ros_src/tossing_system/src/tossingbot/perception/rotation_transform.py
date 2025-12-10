import torch
import numpy as np
import torchvision.transforms.functional as TF

class RotationTransform:
    """
    Central logic for rotating Tensors and Pixels between 
    World Frame and Gripper (Aligned) Frame.
    """
    
    @staticmethod
    def to_gripper_frame(tensor, angle_deg):
        """
        Rotates World Image -> Aligned Gripper View.
        Convention: Rotates by -angle.
        """
        # TF.rotate handles interpolation and padding automatically
        return TF.rotate(tensor, -angle_deg)

    @staticmethod
    def to_world_frame(tensor, angle_deg):
        """
        Rotates Aligned Heatmap -> World View.
        Convention: Rotates by +angle.
        """
        return TF.rotate(tensor, angle_deg)

    @staticmethod
    def rotate_pixel(u, v, angle_deg, h, w, to_gripper_frame=True):
        """
        Rotates pixel (u,v) corresponding to image rotation `angle_deg`.
        
        TF.rotate(img, angle) performs a COUNTER-CLOCKWISE rotation of the image content.
        This means a point at (x,y) moves to (x', y') via standard rotation matrix.
        """
        # Direction: 
        # If we rotated image by 'angle', we want to find where the old pixel went.
        # So we use +angle.
        # If we want to find where a pixel came from, we use -angle.
        
        # In our code: 
        # We did: rot_img = TF.rotate(img, -angle)  (World -> Gripper)
        # So to map a World Pixel -> Gripper Pixel, we rotate by -angle.
        
        eff_angle = -angle_deg if to_gripper_frame else angle_deg
        angle_rad = np.deg2rad(eff_angle)
        
        # Center (Half-pixel precise)
        cy, cx = (h - 1) / 2.0, (w - 1) / 2.0
        
        # Shift to center
        # In image coords: x is col (v), y is row (u)
        # BUT: Y-axis points DOWN in images. Standard math assumes Y points UP.
        # To use standard rotation matrix, we must flip Y, rotate, then flip back.
        # OR: Just know that a +Angle rotation in Image Space looks Clockwise visually.
        
        y = u - cy # Row relative to center
        x = v - cx # Col relative to center
        
        # Standard Matrix (Counter-Clockwise in Math Frame = Clockwise in Image Frame due to Y-flip)
        # x' = x cos - y sin
        # y' = x sin + y cos
        
        # LET'S FIX THE MIRRORING:
        # If your 90 deg result is mirrored, it means we likely got the sin/cos signs flipped relative to the coordinate system.
        
        # Try INVERTING the Y logic for the rotation calculation only
        y_math = -y 
        
        x_new = x * np.cos(angle_rad) - y_math * np.sin(angle_rad)
        y_new_math = x * np.sin(angle_rad) + y_math * np.cos(angle_rad)
        
        y_new = -y_new_math # Flip back to image coords
        
        # Shift back
        v_new = int(round(x_new + cx))
        u_new = int(round(y_new + cy))
        
        # Clamp
        v_new = min(max(v_new, 0), w - 1)
        u_new = min(max(u_new, 0), h - 1)
        
        return u_new, v_new