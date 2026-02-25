import torch
import torch.nn.functional as F
import numpy as np
import torchvision.transforms.functional as TF

from tossingbot import config as cfg


# --- CONFIG PLACEHOLDERS (You will update these numbers later) ---
# R, G, B, Height
TEMP_MEAN = [0.485, 0.456, 0.406, 0.01] 
TEMP_STD  = [0.229, 0.224, 0.225, 0.03]

def normalize_tensor(image_tensor, device):
    """
    Normalizes a (C, H, W) tensor using (image - mean) / std.
    """
    # Create tensors for broadcasting (C, 1, 1)
    mean = torch.tensor(TEMP_MEAN, device=device).view(-1, 1, 1)
    std = torch.tensor(TEMP_STD, device=device).view(-1, 1, 1)
    
    # Epsilon (1e-6) prevents division by zero if std is 0
    return (image_tensor - mean) / (std + 1e-6)

class RotationTransformer:
    def __init__(self, device='cuda'):
        self.device = device

    def rotate_single_with_padding(self, state_tensor, angle_deg):
        """
        Rotates a single image tensor, using padding to preserve corners.
        This is the single, unified method for generating a rotated view.
        """
        if state_tensor.dim() != 3:
            # Assumes a single (C, H, W) tensor
            raise ValueError("Input tensor must be 3D (C, H, W)")

        c, h, w = state_tensor.shape
        diag = int(np.sqrt(h**2 + w**2))
        pad = (diag - w) // 2

        padded = TF.pad(state_tensor, [pad]*4, fill=0)
        
        # NOTE: We use a NEGATIVE angle because TF.rotate rotates the image
        # counter-clockwise for a positive angle. To match our convention where
        # rot_idx > 0 means a CCW rotation of the *world* (and thus a CW rotation
        # of the image), we must pass a negative angle to TF.rotate.
        rot = TF.rotate(padded, -angle_deg)
        
        crop = TF.center_crop(rot, [h, w])
        return crop

    def rotate_pixel(self, u, v, angle_deg, H, W, to_gripper_frame=True):
        """
        Rotates a pixel coordinate (u, v) around the image center.
        u = Row (y), v = Col (x)
        """
        cx, cy = W / 2.0, H / 2.0
        
        # This factor logic is now correct after previous fixes.
        # to_gripper_frame=True rotates the pixel with the image (CW)
        # to_gripper_frame=False rotates it back (CCW)
        factor = 1.0 if to_gripper_frame else -1.0
        rad = np.radians(angle_deg * factor)
        
        x = v - cx
        y = u - cy
        
        new_x = x * np.cos(rad) - y * np.sin(rad)
        new_y = x * np.sin(rad) + y * np.cos(rad)
        
        v_new = int(new_x + cx)
        u_new = int(new_y + cy)
        
        v_new = np.clip(v_new, 0, W-1)
        u_new = np.clip(u_new, 0, H-1)
        
        return u_new, v_new