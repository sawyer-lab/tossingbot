#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm
import numpy as np
import threading

class PerceptionHandler(object):
    def __init__(self):
        rospy.loginfo("Initializing PerceptionHandler...")

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lock = threading.Lock()
        
        # We store structured numpy array directly
        self.points_data = None 

        # Neural Network ROI Configuration (in Base frame meters)
        self.roi_x_min = 0.3   
        self.roi_x_max = 0.8   
        self.roi_y_min = -0.35 
        self.roi_y_max = 0.35  
        
        # Resolution: Matches your Voxel Filter (7mm)
        self.resolution = 0.007 

        # Calculate fixed image size
        self.img_width = int((self.roi_x_max - self.roi_x_min) / self.resolution)
        self.img_height = int((self.roi_y_max - self.roi_y_min) / self.resolution)

        rospy.loginfo("Tensor Target Size: {}x{} pixels".format(self.img_height, self.img_width))

        rospy.Subscriber("voxel_filtered_points", PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("PerceptionHandler initialized.")

    def cb(self, msg):
        try:
            # 1. Transform Cloud to Base Frame
            trans = self.tf_buffer.lookup_transform(
                "base", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1)
            )
            cloud_tf = tf2_sm.do_transform_cloud(msg, trans)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF Error: " + str(e))
            return

        # 2. Robust Parsing (Handles Padding/Different Strides)
        point_step = cloud_tf.point_step
        raw_data = np.frombuffer(cloud_tf.data, dtype=np.uint8)
        
        try:
            raw_data = raw_data.reshape(-1, point_step)
        except ValueError:
            rospy.logwarn("Cloud data corrupted or incomplete frame.")
            return

        # Find offsets
        x_off = next((f.offset for f in cloud_tf.fields if f.name == 'x'), 0)
        y_off = next((f.offset for f in cloud_tf.fields if f.name == 'y'), 4)
        z_off = next((f.offset for f in cloud_tf.fields if f.name == 'z'), 8)
        rgb_off = next((f.offset for f in cloud_tf.fields if f.name == 'rgb'), None)

        if rgb_off is None:
            return

        # 3. FIX: Force contiguous memory before viewing as float
        # We must copy the sliced columns so the bytes are adjacent in memory
        x_bytes = np.ascontiguousarray(raw_data[:, x_off:x_off+4])
        y_bytes = np.ascontiguousarray(raw_data[:, y_off:y_off+4])
        z_bytes = np.ascontiguousarray(raw_data[:, z_off:z_off+4])
        rgb_bytes = np.ascontiguousarray(raw_data[:, rgb_off:rgb_off+4])

        # Now .view() works perfectly
        x = x_bytes.view(np.float32).reshape(-1)
        y = y_bytes.view(np.float32).reshape(-1)
        z = z_bytes.view(np.float32).reshape(-1)
        rgb = rgb_bytes.view(np.float32).reshape(-1)

        # 4. Filter NaNs
        mask = ~np.isnan(x)
        
        # Reconstruct structured array
        cloud_arr = np.zeros(np.sum(mask), dtype=[('x', np.float32), 
                                                  ('y', np.float32), 
                                                  ('z', np.float32), 
                                                  ('rgb', np.float32)])
        cloud_arr['x'] = x[mask]
        cloud_arr['y'] = y[mask]
        cloud_arr['z'] = z[mask]
        cloud_arr['rgb'] = rgb[mask]

        with self.lock:
            self.points_data = cloud_arr


    def get_last_tensor(self):
        """
        Returns a 6-Channel Tensor ready for Neural Network.
        Shape: (Height, Width, 6)
        Channels: [X, Y, Z, R, G, B]
        
        - X, Y, Z are in meters (Base Frame)
        - R, G, B are normalized (0.0 to 1.0)
        """
        with self.lock:
            if self.points_data is None:
                return None
            data = self.points_data.copy()

        # 1. Filter ROI
        mask = (data['x'] >= self.roi_x_min) & (data['x'] < self.roi_x_max) & \
               (data['y'] >= self.roi_y_min) & (data['y'] < self.roi_y_max)
        roi_points = data[mask]

        # Initialize empty tensor (Height, Width, 6)
        # Default value is 0.0. Neural Networks usually prefer 0.0 for "empty space"
        tensor_map = np.zeros((self.img_height, self.img_width, 6), dtype=np.float32)

        if len(roi_points) == 0:
            return tensor_map

        # 2. Compute Grid Indices (Quantization)
        u_indices = ((roi_points['x'] - self.roi_x_min) / self.resolution).astype(int)
        v_indices = ((roi_points['y'] - self.roi_y_min) / self.resolution).astype(int)

        # Clip to bounds
        u_indices = np.clip(u_indices, 0, self.img_width - 1)
        v_indices = np.clip(v_indices, 0, self.img_height - 1)

        # 3. Z-Buffering (Handling overlaps)
        # We sort by Z so the highest point "wins" the pixel
        sort_indices = np.argsort(roi_points['z'])
        
        sorted_u = u_indices[sort_indices]
        sorted_v = v_indices[sort_indices]
        
        # Sorted Data
        s_x = roi_points['x'][sort_indices]
        s_y = roi_points['y'][sort_indices]
        s_z = roi_points['z'][sort_indices]
        s_rgb = roi_points['rgb'][sort_indices]

        # 4. Fill Channels 0, 1, 2 (X, Y, Z)
        # Note: We fill the pixel with the ACTUAL metric coordinate of the point,
        # not the center of the pixel. This is more precise.
        tensor_map[sorted_v, sorted_u, 0] = s_x
        tensor_map[sorted_v, sorted_u, 1] = s_y
        tensor_map[sorted_v, sorted_u, 2] = s_z

        # 5. Fill Channels 3, 4, 5 (R, G, B)
        # Unpack float32 RGB to uint8
        rgb_uint8 = s_rgb.view(np.uint8).reshape(-1, 4)
        
        # Normalize to 0.0 - 1.0 range
        b_norm = rgb_uint8[:, 0].astype(np.float32) / 255.0
        g_norm = rgb_uint8[:, 1].astype(np.float32) / 255.0
        r_norm = rgb_uint8[:, 2].astype(np.float32) / 255.0

        tensor_map[sorted_v, sorted_u, 3] = r_norm
        tensor_map[sorted_v, sorted_u, 4] = g_norm
        tensor_map[sorted_v, sorted_u, 5] = b_norm

        return tensor_map

if __name__ == "__main__":
    rospy.init_node("perception_tensor_test")
    handler = PerceptionHandler()
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        tensor = handler.get_last_tensor()
        # Inside your while loop:
        if tensor is not None:
            # Get the Z channel (index 2)
            z_channel = tensor[:, :, 2]
            
            # Calculate stats ignoring zeros (empty space)
            valid_z = z_channel[z_channel != 0]
            
            # if len(valid_z) > 0:
            #     print("Min Height: {:.3f}m | Max Height: {:.3f}m".format(np.min(valid_z), np.max(valid_z)))
            # else:
            #     print("Tensor is empty (Camera looking at nothing?)")
            
        r.sleep()