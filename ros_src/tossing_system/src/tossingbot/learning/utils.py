import torch
import torch.nn.functional as F
import numpy as np

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

# --- 1. Tensor Rotations ---
def create_rotated_batch(image_tensor, num_rotations, device):
    """
    Takes a single (C, H, W) tensor, normalizes it, 
    and returns a batch (N, C, H, W) of rotated versions.
    """
    image_tensor = image_tensor.to(device)
    
    # --- STEP 1: NORMALIZE BEFORE ROTATING ---
    norm_img = normalize_tensor(image_tensor, device)
    
    B, C, H, W = 1, norm_img.shape[0], norm_img.shape[1], norm_img.shape[2]
    
    rotated_list = []
    
    step = 180.0 / num_rotations
    
    for i in range(num_rotations):
        angle_deg = i * step
        # Negative because affine_grid rotates the sampling grid, effectively rotating image opposite
        theta = -np.radians(angle_deg) 

        rot_mat = torch.tensor([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0]
        ], dtype=torch.float32, device=device).unsqueeze(0) # (1, 2, 3)

        grid = F.affine_grid(rot_mat, torch.Size((1, C, H, W)), align_corners=True)
        
        # Use the NORMALIZED image here
        rot_img = F.grid_sample(norm_img.unsqueeze(0), grid, align_corners=True)
        
        rotated_list.append(rot_img.squeeze(0))

    return torch.stack(rotated_list)

class RotationTransformer:
    def __init__(self, device='cuda'):
        self.device = device

    def to_gripper_frame(self, state_tensor, angle_deg):
        """
        Rotates the image TENSOR by 'angle_deg'.
        Used to simulate the gripper rotating relative to the object.
        """
        # Ensure 4 dimensions (B, C, H, W)
        if state_tensor.dim() == 3:
            state_tensor = state_tensor.unsqueeze(0)
            
        B, C, H, W = state_tensor.shape
        
        theta = -np.radians(angle_deg)
        
        rot_mat = torch.tensor([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0]
        ], dtype=torch.float32, device=self.device).unsqueeze(0)
        
        if B > 1:
            rot_mat = rot_mat.repeat(B, 1, 1)

        grid = F.affine_grid(rot_mat, torch.Size((B, C, H, W)), align_corners=True)
        rot_img = F.grid_sample(state_tensor, grid, align_corners=True, mode='nearest')
        
        return rot_img.squeeze(0) 

    def to_world_frame(self, state_tensor, angle_deg):
        """Inverse of to_gripper_frame."""
        return self.to_gripper_frame(state_tensor, -angle_deg)

    def rotate_pixel(self, u, v, angle_deg, H, W, to_gripper_frame=True):
        """
        Rotates a pixel coordinate (u, v) around the image center.
        u = Row (y), v = Col (x)
        """
        cx, cy = W / 2.0, H / 2.0
        
        factor = -1.0 if to_gripper_frame else 1.0
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