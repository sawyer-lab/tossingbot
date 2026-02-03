#!/usr/bin/env python3.8
import rospy
import sys
import numpy as np
import open3d as o3d
import torch
import message_filters
import copy
import ros_numpy
from sensor_msgs.msg import PointCloud2, Image
from tossingbot import config

PUBLISH_DEBUG = False

class PerceptionModule:
    def __init__(self, tf_buffer, ):
        self.tf_buffer = tf_buffer
        self.latest_pcd_combined = None

        
        # Debug Publishers
        if PUBLISH_DEBUG:
            self.pub_step1 = rospy.Publisher("/debug/1_merged", PointCloud2, queue_size=1)
            self.pub_step2 = rospy.Publisher("/debug/2_cropped", PointCloud2, queue_size=1)
            self.pub_tensor = rospy.Publisher("/debug/tensor_img", Image, queue_size=1)

        # Sync Subscribers
        left_sub = message_filters.Subscriber("/rgbd_camera_left/depth/points", PointCloud2)
        right_sub = message_filters.Subscriber("/rgbd_camera_right/depth/points", PointCloud2)
        
        # Increased slop to 0.2 to catch unsynced frames in simulation
        self.ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self._cb_clouds_merged)
        
        rospy.loginfo("Perception Module listening for PointClouds...")

    def get_observation_tensor(self):
        """Returns (3, H, W) torch tensor or None"""
        if self.latest_pcd_combined is None: 
            return None
        
        pcd = copy.deepcopy(self.latest_pcd_combined)

        # 1. Crop
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[config.ROI_X[0], config.ROI_Y[0], config.ROI_Z[0]], 
            max_bound=[config.ROI_X[1], config.ROI_Y[1], config.ROI_Z[1]]
        )
        pcd = pcd.crop(bbox)
        
        if len(pcd.points) == 0: 
            return None
        
        # 2. Voxel
        pcd = pcd.voxel_down_sample(voxel_size=config.VOXEL_SIZE)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step2)

        # 3. Project to Image
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        u = ((xyz[:, 1] - config.ROI_Y[0]) / config.GRID_RES).astype(int) 
        v = ((xyz[:, 0] - config.ROI_X[0]) / config.GRID_RES).astype(int) 
        u = np.clip(u, 0, config.IMG_H - 1)
        v = np.clip(v, 0, config.IMG_W - 1)

        tensor_map = np.zeros((config.IMG_H, config.IMG_W, 3), dtype=np.float32)
        
        # Z-Sort (Highest points render on top)
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        tensor_map[u, v] = rgb[sort_idx]

        if PUBLISH_DEBUG:
            img_uint8 = (tensor_map * 255).astype(np.uint8)
            msg = ros_numpy.msgify(Image, img_uint8, encoding='rgb8')
            msg.header.frame_id = "base"
            self.pub_tensor.publish(msg)

        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def pixel_to_world(self, u, v):
        world_x = config.ROI_X[0] + (v * config.GRID_RES) + (config.GRID_RES / 2.0)
        world_y = config.ROI_Y[0] + (u * config.GRID_RES) + (config.GRID_RES / 2.0)
        world_z = 0.03
        return np.array([world_x, world_y, world_z])

    # --- Helpers ---
    def _cb_clouds_merged(self, msg_left, msg_right):
        # rospy.loginfo("Callback Triggered!") # Uncomment if still stuck
        pcd_left = self._process_cloud(msg_left)
        pcd_right = self._process_cloud(msg_right)
        
        if pcd_left and pcd_right:
            self.latest_pcd_combined = pcd_left + pcd_right
            if PUBLISH_DEBUG: 
                self.publish_o3d(self.latest_pcd_combined, self.pub_step1)

    def _process_cloud(self, msg):
        try:
            pc_np = ros_numpy.numpify(msg).reshape(-1)
            points = np.zeros((pc_np.shape[0], 3), dtype=np.float64)
            points[:,0], points[:,1], points[:,2] = pc_np['x'], pc_np['y'], pc_np['z']
            
            rgb_f32 = pc_np['rgb']
            rgb_u32 = rgb_f32.view(np.uint32)
            r = ((rgb_u32 >> 16) & 0xFF) / 255.0
            g = ((rgb_u32 >> 8) & 0xFF) / 255.0
            b = (rgb_u32 & 0xFF) / 255.0
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(np.stack([r,g,b], axis=-1))
            
            # Transform to base
            trans = self.tf_buffer.lookup_transform("base", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
            R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            
            # Apply transformation
            pcd.transform(np.vstack((np.hstack((R, np.array(t).reshape(3,1))), [0,0,0,1])))
            return pcd
            
        except Exception as e:
            # PRINT THE ERROR so you know why it's failing
            rospy.logwarn(f"Perception Error: {e}") 
            return None

    def publish_o3d(self, pcd, publisher):
        if len(pcd.points) == 0: return
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        r = (colors[:, 0] * 255).astype(np.uint32)
        g = (colors[:, 1] * 255).astype(np.uint32)
        b = (colors[:, 2] * 255).astype(np.uint32)
        rgb_float = ((b << 16) | (g << 8) | r).view(np.float32)

        data = np.zeros(len(points), dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
        data['x'], data['y'], data['z'], data['rgb'] = points[:,0], points[:,1], points[:,2], rgb_float
        
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)