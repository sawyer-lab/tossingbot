#!/usr/bin/env python3.8
import sys
import os

# ==============================================================================
# CRITICAL: LOAD CUSTOM COMPILED LIBRARIES
# ==============================================================================
# These point to the workspaces we built manually in the Dockerfile
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import numpy as np
import threading
import torch
import torch.nn as nn

# --- Helper: Pure NumPy Quaternion to Rotation Matrix ---
# We use this to avoid importing tf2_sensor_msgs or PyKDL
def get_transformation_matrix(trans):
    tx = trans.transform.translation.x
    ty = trans.transform.translation.y
    tz = trans.transform.translation.z
    t_vec = np.array([tx, ty, tz], dtype=np.float32)

    qx = trans.transform.rotation.x
    qy = trans.transform.rotation.y
    qz = trans.transform.rotation.z
    qw = trans.transform.rotation.w

    # Construct Rotation Matrix from Quaternion
    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx*qy - qz*qw)
    r02 = 2 * (qx*qz + qy*qw)
    
    r10 = 2 * (qx*qy + qz*qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy*qz - qx*qw)
    
    r20 = 2 * (qx*qz - qy*qw)
    r21 = 2 * (qy*qz + qx*qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    R_mat = np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ], dtype=np.float32)
    
    return t_vec, R_mat

class PerceptionAINode(object):
    def __init__(self):
        rospy.init_node("perception_ai_node")
        rospy.loginfo("Initializing Python 3.8 AI Node...")

        # --- GPU SETUP ---
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo(f"--- DEVICE: {self.device} ---")
        
        if self.device.type == 'cuda':
            rospy.loginfo(f"GPU Model: {torch.cuda.get_device_name(0)}")
        else:
            rospy.logwarn("Running on CPU! Check Docker GPU runtime settings.")

        # --- MODEL LOAD ---
        # Replace this with: self.model = torch.load('path/to/weights.pt')
        self.model = self.load_dummy_model()
        self.model.to(self.device)
        self.model.eval()

        # --- ROS SETUP ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.Lock()
        self.points_data = None 

        # --- CONFIG ---
        self.roi_x_min, self.roi_x_max = 0.0, 0.8   
        self.roi_y_min, self.roi_y_max = -0.35, 0.35  
        self.resolution = 0.007 

        self.img_width = int((self.roi_x_max - self.roi_x_min) / self.resolution)
        self.img_height = int((self.roi_y_max - self.roi_y_min) / self.resolution)
        
        rospy.loginfo(f"Tensor Shape: (6, {self.img_height}, {self.img_width})")

        rospy.Subscriber("voxel_filtered_points", PointCloud2, self.cb, queue_size=1)

    def load_dummy_model(self):
        # A simple conv layer just to prove GPU inference works
        return nn.Conv2d(in_channels=6, out_channels=1, kernel_size=3, padding=1)

    def cb(self, msg):
        # 1. Get Transform
        try:
            trans = self.tf_buffer.lookup_transform("base", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return

        # 2. Parse Raw Data (Camera Frame)
        point_step = msg.point_step
        raw_data = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            raw_data = raw_data.reshape(-1, point_step)
        except ValueError:
            return

        # Get Field Offsets
        x_off = next((f.offset for f in msg.fields if f.name == 'x'), 0)
        y_off = next((f.offset for f in msg.fields if f.name == 'y'), 4)
        z_off = next((f.offset for f in msg.fields if f.name == 'z'), 8)
        rgb_off = next((f.offset for f in msg.fields if f.name == 'rgb'), None)

        if rgb_off is None: return

        # Extract & Cast to Float (N, )
        x = np.ascontiguousarray(raw_data[:, x_off:x_off+4]).view(np.float32).reshape(-1)
        y = np.ascontiguousarray(raw_data[:, y_off:y_off+4]).view(np.float32).reshape(-1)
        z = np.ascontiguousarray(raw_data[:, z_off:z_off+4]).view(np.float32).reshape(-1)
        rgb = np.ascontiguousarray(raw_data[:, rgb_off:rgb_off+4]).view(np.float32).reshape(-1)

        # 3. Apply Transform (Manually)
        t_vec, R_mat = get_transformation_matrix(trans)
        points_camera = np.vstack((x, y, z)).T 
        
        # P_base = R * P_cam + T
        points_base = np.dot(points_camera, R_mat.T) + t_vec

        x_base = points_base[:, 0]
        y_base = points_base[:, 1]
        z_base = points_base[:, 2]

        # 4. Store Result
        mask = ~np.isnan(x_base)
        cloud_arr = np.zeros(np.sum(mask), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
        cloud_arr['x'] = x_base[mask]
        cloud_arr['y'] = y_base[mask]
        cloud_arr['z'] = z_base[mask]
        cloud_arr['rgb'] = rgb[mask]

        with self.lock:
            self.points_data = cloud_arr

    def generate_tensor(self):
        with self.lock:
            if self.points_data is None: return None
            data = self.points_data

        # ROI Filter
        mask = (data['x'] >= self.roi_x_min) & (data['x'] < self.roi_x_max) & \
               (data['y'] >= self.roi_y_min) & (data['y'] < self.roi_y_max)
        roi_points = data[mask]

        tensor_map = np.zeros((self.img_height, self.img_width, 6), dtype=np.float32)
        if len(roi_points) == 0: return tensor_map

        # Quantization
        u = ((roi_points['x'] - self.roi_x_min) / self.resolution).astype(int)
        v = ((roi_points['y'] - self.roi_y_min) / self.resolution).astype(int)
        
        u = np.clip(u, 0, self.img_width - 1)
        v = np.clip(v, 0, self.img_height - 1)

        # Z-Buffer
        sort_idx = np.argsort(roi_points['z'])
        u, v = u[sort_idx], v[sort_idx]
        p_sorted = roi_points[sort_idx]

        # Fill Geometry
        tensor_map[v, u, 0] = p_sorted['x']
        tensor_map[v, u, 1] = p_sorted['y']
        tensor_map[v, u, 2] = p_sorted['z']

        # Fill Color (FIXED CONTIGUOUS ERROR HERE)
        # We enforce contiguous memory before viewing as uint8
        rgb_bytes = np.ascontiguousarray(p_sorted['rgb'])
        rgb_u8 = rgb_bytes.view(np.uint8).reshape(-1, 4)
        
        tensor_map[v, u, 3] = rgb_u8[:, 2].astype(np.float32) / 255.0 # R
        tensor_map[v, u, 4] = rgb_u8[:, 1].astype(np.float32) / 255.0 # G
        tensor_map[v, u, 5] = rgb_u8[:, 0].astype(np.float32) / 255.0 # B

        return tensor_map

    def run_inference(self):
        np_tensor = self.generate_tensor()
        if np_tensor is None: return

        # Convert to PyTorch (Batch, Channel, Height, Width)
        tensor_torch = torch.from_numpy(np_tensor).permute(2, 0, 1).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(tensor_torch)
        
        # print(f"Inference Done. Output Shape: {output.shape}")

if __name__ == "__main__":
    try:
        node = PerceptionAINode()
        rate = rospy.Rate(30) # Run at 30Hz
        while not rospy.is_shutdown():
            node.run_inference()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass