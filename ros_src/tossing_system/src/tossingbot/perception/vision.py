import numpy as np
import open3d as o3d
import torch
from tossingbot import config

class VisionProcessor:
    """
    Pure Logic: Converts Raw PointClouds -> Training Tensors.
    Zero dependencies on ROS or Hardware.
    """
    def __init__(self):
        # Pre-compute bbox for speed
        self.bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[config.ROI_X[0], config.ROI_Y[0], config.ROI_Z[0]], 
            max_bound=[config.ROI_X[1], config.ROI_Y[1], config.ROI_Z[1]]
        )

    def process(self, points, colors):
        """
        Args:
            points: np.array (N, 3)
            colors: np.array (N, 3)
        Returns:
            torch.Tensor: (3, H, W) or None if empty
        """
        if len(points) == 0: return None
        
        # 1. Create Open3D Object (Efficient C++ Wrapper)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # 2. Crop
        pcd = pcd.crop(self.bbox)
        if len(pcd.points) == 0: return None

        # 3. Voxel Downsample (Critical for fusing overlaps)
        pcd = pcd.voxel_down_sample(voxel_size=config.VOXEL_SIZE)
        
        # 4. Project to Tensor
        return self._project_to_grid(pcd)

    def _project_to_grid(self, pcd):
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        # Map World (Metric) -> Grid (Indices)
        u = ((xyz[:, 1] - config.ROI_Y[0]) / config.GRID_RES).astype(int)
        v = ((xyz[:, 0] - config.ROI_X[0]) / config.GRID_RES).astype(int)
        
        # Clip to bounds
        u = np.clip(u, 0, config.IMG_H - 1)
        v = np.clip(v, 0, config.IMG_W - 1)

        # Initialize Image Buffer
        tensor_map = np.zeros((config.IMG_H, config.IMG_W, 3), dtype=np.float32)
        
        # Z-Buffer Sort (Render highest Z points on top)
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        
        # Paint pixels
        tensor_map[u, v] = rgb[sort_idx]

        # Convert to Torch (Channels, Height, Width)
        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def pixel_to_world(self, u, v):
        """Used by the Agent to interpret action."""
        world_x = config.ROI_X[0] + (v * config.GRID_RES) + (config.GRID_RES / 2.0)
        world_y = config.ROI_Y[0] + (u * config.GRID_RES) + (config.GRID_RES / 2.0)
        # Return [x, y, z] (Using Fixed Grasp Height from Config)
        return np.array([world_x, world_y, config.GRASP_Z])