import numpy as np
import open3d as o3d
import torch
from tossingbot import config

class VisionProcessor:
    def __init__(self):
        # Crop bounds from config
        self.min_x, self.max_x = config.ROI_X[0], config.ROI_X[1]
        self.min_y, self.max_y = config.ROI_Y[0], config.ROI_Y[1]
        
        self.bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[self.min_x, self.min_y, config.ROI_Z[0]], 
            max_bound=[self.max_x, self.max_y, config.ROI_Z[1]]
        )

    def process(self, points, colors):
        if len(points) == 0: return None
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        pcd = pcd.crop(self.bbox)
        if len(pcd.points) == 0: return None

        pcd = pcd.voxel_down_sample(voxel_size=config.VOXEL_SIZE)
        return self._project_to_grid(pcd)

    def _project_to_grid(self, pcd):
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        x_range = self.max_x - self.min_x
        u = ((self.max_x - xyz[:, 0]) / x_range * config.IMG_H).astype(int)

        y_range = self.max_y - self.min_y
        v = ((self.max_y - xyz[:, 1]) / y_range * config.IMG_W).astype(int)

        u = np.clip(u, 0, config.IMG_H - 1)
        v = np.clip(v, 0, config.IMG_W - 1)

        tensor_map = np.zeros((config.IMG_H, config.IMG_W, 4), dtype=np.float32)
        
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        sorted_xyz = xyz[sort_idx]
        sorted_rgb = rgb[sort_idx]
        
        tensor_map[u, v, :3] = sorted_rgb
        
        height_vals = sorted_xyz[:, 2] - config.TABLE_HEIGHT
        tensor_map[u, v, 3] = np.clip(height_vals, 0.0, 1.0) 

        # Permute to (C, H, W) -> (4, H, W)
        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def pixel_to_world(self, u, v):

        pct_u = float(u) / config.IMG_H
        world_x = self.max_x - (pct_u * (self.max_x - self.min_x))
        

        pct_v = float(v) / config.IMG_W
        world_y = self.max_y - (pct_v * (self.max_y - self.min_y))

        return np.array([world_x, world_y, config.GRASP_Z])