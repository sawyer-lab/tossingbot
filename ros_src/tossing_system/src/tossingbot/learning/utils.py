import torch
import torch.nn.functional as F
import numpy as np

# --- 1. Tensor Rotations ---
def create_rotated_batch(image_tensor, num_rotations, device):
    """
    Takes a single (C, H, W) tensor and returns a batch (N, C, H, W)
    of rotated versions.
    """
    image_tensor = image_tensor.to(device)
    B, C, H, W = 1, image_tensor.shape[0], image_tensor.shape[1], image_tensor.shape[2]
    
    rotated_list = []
    
    step = 180.0 / num_rotations
    
    for i in range(num_rotations):
        angle_deg = i * step
        theta = -np.radians(angle_deg)
        

        rot_mat = torch.tensor([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0]
        ], dtype=torch.float32, device=device).unsqueeze(0) # (1, 2, 3)

        grid = F.affine_grid(rot_mat, torch.Size((1, C, H, W)), align_corners=True)
        rot_img = F.grid_sample(image_tensor.unsqueeze(0), grid, align_corners=True)
        
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
        
        # PyTorch affine_grid expects Radians
        # Negative angle because Y-axis is inverted in image coordinates usually, 
        # or to match the "camera rotates vs object rotates" logic.
        theta = -np.radians(angle_deg)
        
        # Affine Matrix [ cos -sin  0 ]
        #               [ sin  cos  0 ]
        rot_mat = torch.tensor([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0]
        ], dtype=torch.float32, device=self.device).unsqueeze(0)
        
        # If batch size > 1, repeat matrix
        if B > 1:
            rot_mat = rot_mat.repeat(B, 1, 1)

        grid = F.affine_grid(rot_mat, torch.Size((B, C, H, W)), align_corners=True)
        rot_img = F.grid_sample(state_tensor, grid, align_corners=True, mode='nearest')
        
        return rot_img.squeeze(0) # Return (C, H, W)

    def to_world_frame(self, state_tensor, angle_deg):
        """Inverse of to_gripper_frame."""
        return self.to_gripper_frame(state_tensor, -angle_deg)

    def rotate_pixel(self, u, v, angle_deg, H, W, to_gripper_frame=True):
        """
        Rotates a pixel coordinate (u, v) around the image center.
        u = Row (y), v = Col (x)
        """
        cx, cy = W / 2.0, H / 2.0
        
        # If to_gripper (World -> Image), we rotate by -angle
        # If to_world (Image -> World), we rotate by +angle
        factor = -1.0 if to_gripper_frame else 1.0
        rad = np.radians(angle_deg * factor)
        
        # Translate to Center
        x = v - cx
        y = u - cy
        
        # Rotation Matrix
        # x' = x cos - y sin
        # y' = x sin + y cos
        new_x = x * np.cos(rad) - y * np.sin(rad)
        new_y = x * np.sin(rad) + y * np.cos(rad)
        
        # Translate back
        v_new = int(new_x + cx)
        u_new = int(new_y + cy)
        
        # Clamp
        v_new = np.clip(v_new, 0, W-1)
        u_new = np.clip(u_new, 0, H-1)
        
        return u_new, v_new