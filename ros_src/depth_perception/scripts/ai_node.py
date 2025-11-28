#!/usr/bin/env python3.8
import sys
import os

# --- PATHS ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')
import open3d as o3d
import torch
import torch.nn as nn
import rospy
import tf2_ros
import numpy as np
import ros_numpy


from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# Set this to True to pause execution and open a window at every step
DEBUG_STEP_BY_STEP = False 

# Set this to True to publish intermediate steps to Rviz (Real-time)
PUBLISH_DEBUG_TOPICS = True
# ==============================================================================

class PerceptionDebugger:
    def __init__(self):
        rospy.init_node("perception_debugger")
        
        # GPU Setup
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # GPU Setup
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # --- ADD THIS BLOCK ---
        rospy.loginfo(f"Selected Device: {self.device}")
        if self.device.type == 'cuda':
            rospy.loginfo(f"GPU Name: {torch.cuda.get_device_name(0)}")
            rospy.loginfo(f"CUDA Version: {torch.version.cuda}")
        else:
            rospy.logwarn("Running on CPU! CUDA is not available.")
        # ----------------------

        self.model = nn.Conv2d(6, 1, kernel_size=3, padding=1).to(self.device)

        # self.device = torch.device('cpu')  # For debugging, we use CPU
        self.model = nn.Conv2d(6, 1, kernel_size=3, padding=1).to(self.device) # Dummy model

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Pipeline Config
        self.voxel_size = 0.005 
        self.roi_bounds = np.array([0.0, -0.40, -0.1, 0.9, 0.40, 0.5]) 
        self.grid_res = 0.007 
        self.img_width = int((self.roi_bounds[3] - self.roi_bounds[0]) / self.grid_res)
        self.img_height = int((self.roi_bounds[4] - self.roi_bounds[1]) / self.grid_res)

        # --- DEBUG PUBLISHERS ---
        if PUBLISH_DEBUG_TOPICS:
            self.pub_step1 = rospy.Publisher("/debug/1_raw_transformed", PointCloud2, queue_size=1)
            self.pub_step2 = rospy.Publisher("/debug/2_cropped", PointCloud2, queue_size=1)
            self.pub_step3 = rospy.Publisher("/debug/3_voxelized", PointCloud2, queue_size=1)
            self.pub_tensor = rospy.Publisher("/debug/4_tensor_rgb", Image, queue_size=1)

        rospy.Subscriber("/rgbd_camera/depth/points", PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("Debug Node Started. Waiting for point clouds...")

    def visualize_step(self, geometry, title="Debug"):
        """
        Pauses code and opens an Open3D window.
        """
        if DEBUG_STEP_BY_STEP:
            print(f"[DEBUG] Visualizing: {title} (Close window to continue)")
            # Add a coordinate frame for reference
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            o3d.visualization.draw_geometries([geometry, axis], window_name=title)

    def publish_o3d(self, pcd, publisher, frame_id):
        """
        Helper: Converts Open3D -> ROS PointCloud2 for Rviz
        """
        if not PUBLISH_DEBUG_TOPICS: return
        
        points = np.asarray(pcd.points)
        if len(points) == 0: return
        
        colors = np.asarray(pcd.colors) # Float 0..1
        
        # Create structured array for ros_numpy
        # We pack RGB float back into the specific ROS structure
        data = np.zeros(len(points), dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)
        ])
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['r'] = (colors[:, 0] * 255).astype(np.uint8)
        data['g'] = (colors[:, 1] * 255).astype(np.uint8)
        data['b'] = (colors[:, 2] * 255).astype(np.uint8)

        # Use ros_numpy to create the message
        # Note: We manually create the 'rgb' float field usually, but splitting r,g,b works in Rviz too
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)

    def cb(self, msg):
        # -----------------------------------------------------------
        # STEP 0: Parse Input
        # -----------------------------------------------------------
        try:
            pc_np = ros_numpy.numpify(msg)
        except Exception: 
            return

        # === FIX: FLATTEN THE CLOUD ===
        # Reshape (Height, Width) -> (Total_Points,)
        pc_np = pc_np.reshape(-1)
        # ==============================

        # Now extract coordinates
        points = np.zeros((pc_np.shape[0], 3), dtype=np.float64)
        points[:,0] = pc_np['x']
        points[:,1] = pc_np['y']
        points[:,2] = pc_np['z']
        
        # Filter NaNs (Crucial for Depth Cameras)
        valid_mask = ~np.isnan(points).any(axis=1)
        points = points[valid_mask]
        
        if len(points) == 0: return

        # Handle RGB Packing
        # (This logic remains the same, but now operates on the flattened array)
        rgb_f32 = pc_np['rgb'][valid_mask]
        rgb_u32 = rgb_f32.view(np.uint32)
        r = ((rgb_u32 >> 16) & 0xFF) / 255.0
        g = ((rgb_u32 >> 8) & 0xFF) / 255.0
        b = (rgb_u32 & 0xFF) / 255.0
        colors = np.stack([r, g, b], axis=-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # ... (Rest of the function remains identical) ...

        # -----------------------------------------------------------
        # STEP 1: Transform to Base
        # -----------------------------------------------------------
        try:
            trans = self.tf_buffer.lookup_transform("base", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            t_vec = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
            R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            T = np.eye(4)
            T[:3, :3] = R; T[:3, 3] = t_vec
            pcd.transform(T)
        except Exception: return

        # DEBUG STEP 1
        self.publish_o3d(pcd, self.pub_step1, "base")
        self.visualize_step(pcd, "Step 1: Transformed Cloud")

        # -----------------------------------------------------------
        # STEP 2: Crop to ROI
        # -----------------------------------------------------------
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=self.roi_bounds[:3], 
            max_bound=self.roi_bounds[3:]
        )
        # To help visualize, let's create a wireframe of the box
        bbox.color = (1, 0, 0) # Red box
        
        pcd_cropped = pcd.crop(bbox)
        
        # DEBUG STEP 2
        self.publish_o3d(pcd_cropped, self.pub_step2, "base")
        if DEBUG_STEP_BY_STEP:
             # Show the cropped cloud AND the bounding box
            self.visualize_step(pcd_cropped, "Step 2: Cropped (Red Box is ROI)")

        if len(pcd_cropped.points) == 0: return

        # -----------------------------------------------------------
        # STEP 3: Voxelization & Outlier Removal
        # -----------------------------------------------------------
        pcd_down = pcd_cropped.voxel_down_sample(voxel_size=self.voxel_size)
        pcd_clean, _ = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # DEBUG STEP 3
        self.publish_o3d(pcd_clean, self.pub_step3, "base")
        self.visualize_step(pcd_clean, "Step 3: Voxelized & Cleaned")

        # -----------------------------------------------------------
        # STEP 4: Tensor Generation (Projection)
        # -----------------------------------------------------------
        # (This logic converts 3D -> 2D Grid)
        xyz = np.asarray(pcd_clean.points)
        rgb = np.asarray(pcd_clean.colors)

        u = ((xyz[:, 0] - self.roi_bounds[0]) / self.grid_res).astype(int)
        v = ((xyz[:, 1] - self.roi_bounds[1]) / self.grid_res).astype(int)
        u = np.clip(u, 0, self.img_width - 1)
        v = np.clip(v, 0, self.img_height - 1)

        tensor_map = np.zeros((self.img_height, self.img_width, 6), dtype=np.float32)
        
        # Sort by Z
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        tensor_map[v, u, 0:3] = xyz[sort_idx]
        tensor_map[v, u, 3:6] = rgb[sort_idx]

        # DEBUG STEP 4: Visualize the RGB part of the tensor
        if PUBLISH_DEBUG_TOPICS:
            # Extract RGB channels (3,4,5), scale to 0-255 uint8
            rgb_img = (tensor_map[:, :, 3:6] * 255).astype(np.uint8)
            # Create ROS Image
            img_msg = ros_numpy.msgify(Image, rgb_img, encoding='rgb8')
            self.pub_tensor.publish(img_msg)

if __name__ == "__main__":
    node = PerceptionDebugger()
    rospy.spin()