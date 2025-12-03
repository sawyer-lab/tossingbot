#!/usr/bin/env python3.8
import sys

# 1. PYTHON 3 COMPATIBILITY HACKS
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import open3d as o3d
import torch
import message_filters
import copy
import cv2

from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# Bounds (Robot Frame)
ROI_X = [0.65, 1.0]    
ROI_Y = [-0.3, 0.3] 
ROI_Z = [-0.1, 0.5]   

# Resolution
VOXEL_SIZE = 0.005
GRID_RES = 0.005 

# Calculated Dimensions
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

PUBLISH_DEBUG = True
OBJECT_NAME = "spoon"
# ==============================================================================

class GazeboEnv:
    def __init__(self):
        if rospy.get_node_uri() is None:
            rospy.init_node('gazebo_env_node')

        rospy.loginfo("Initializing Gazebo Environment (Dual Camera + RL Services)...")

        # 1. ROS Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # 2. Perception Input (Dual Camera Sync)
        self.latest_pcd_combined = None 

        # --- SERVICE PROXIES ---
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # Subscribe to both topics
        left_sub = message_filters.Subscriber("/rgbd_camera_left/depth/points", PointCloud2)
        right_sub = message_filters.Subscriber("/rgbd_camera_right/depth/points", PointCloud2)
        
        # Sync
        self.ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self._cb_clouds_merged)

        # 3. Debug Publishers
        if PUBLISH_DEBUG:
            self.pub_step1 = rospy.Publisher("/debug/1_merged_transformed", PointCloud2, queue_size=1)
            self.pub_step2 = rospy.Publisher("/debug/2_cropped", PointCloud2, queue_size=1)
            self.pub_step3 = rospy.Publisher("/debug/3_clean", PointCloud2, queue_size=1)
            self.pub_tensor = rospy.Publisher("/debug/4_tensor_img", Image, queue_size=1)

        rospy.loginfo("Env Ready. Waiting for synced PointClouds...")
        while self.latest_pcd_combined is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Dual Camera Stream Received!")

    # --------------------------------------------------------------------------
    # SERVICES
    # --------------------------------------------------------------------------
    def _reset_spoon(self):
        state_msg = ModelState()
        state_msg.model_name = OBJECT_NAME
        state_msg.reference_frame = "world"
        
        rand_x = np.random.uniform(ROI_X[0]+0.1, ROI_X[1]-0.1)
        rand_y = np.random.uniform(ROI_Y[0]+0.1, ROI_Y[1]-0.1)
        
        state_msg.pose.position = Point(rand_x, rand_y, 0.8)
        state_msg.pose.orientation = Quaternion(-0.00374595104945, 0.0031389867495, -0.648059770085, 0.761573797482)

        try:
            self.set_state_srv(state_msg)
            rospy.sleep(0.5) 
        except rospy.ServiceException as e:
            rospy.logwarn(f"Reset failed: {e}")

    def _check_success(self):
        try:
            resp = self.get_state_srv(OBJECT_NAME, "world")
            if not resp.success: return False
            return resp.pose.position.z > 0.90
        except rospy.ServiceException as e:
            return False

    # --------------------------------------------------------------------------
    # PERCEPTION HELPERS
    # --------------------------------------------------------------------------
    def _ros_to_o3d(self, msg):
        try:
            pc_np = ros_numpy.numpify(msg).reshape(-1)
        except Exception: return None

        points = np.zeros((pc_np.shape[0], 3), dtype=np.float64)
        points[:,0], points[:,1], points[:,2] = pc_np['x'], pc_np['y'], pc_np['z']
        
        rgb_f32 = pc_np['rgb']
        rgb_u32 = rgb_f32.view(np.uint32)
        r = ((rgb_u32 >> 16) & 0xFF) / 255.0
        g = ((rgb_u32 >> 8) & 0xFF) / 255.0
        b = (rgb_u32 & 0xFF) / 255.0
        colors = np.stack([r, g, b], axis=-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def _transform_cloud(self, pcd, frame_id):
        try:
            trans = self.tf_buffer.lookup_transform("base", frame_id, rospy.Time(0), rospy.Duration(0.1))
            t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
            R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            T = np.eye(4); T[:3,:3]=R; T[:3,3]=t
            pcd.transform(T)
            return pcd
        except Exception as e:
            return None

    def _cb_clouds_merged(self, msg_left, msg_right):
        pcd_left = self._ros_to_o3d(msg_left)
        pcd_left = self._transform_cloud(pcd_left, msg_left.header.frame_id)
        
        pcd_right = self._ros_to_o3d(msg_right)
        pcd_right = self._transform_cloud(pcd_right, msg_right.header.frame_id)

        if pcd_left and pcd_right:
            self.latest_pcd_combined = pcd_left + pcd_right
            if PUBLISH_DEBUG:
                self.publish_o3d(self.latest_pcd_combined, self.pub_step1)

    # --------------------------------------------------------------------------
    # RL INTERFACE
    # --------------------------------------------------------------------------

    def pixel_to_world(self, u, v):
      
        world_x = ROI_X[0] + (v * GRID_RES) + (GRID_RES / 2.0)
        world_y = ROI_Y[0] + (u * GRID_RES) + (GRID_RES / 2.0)
        

        world_z = 0.78
        
        return np.array([world_x, world_y, world_z])

    def reset(self):
        self._reset_spoon()
        return self.get_observation()

    def step(self, pixel_u, pixel_v):
        target_pos = self.pixel_to_world(pixel_u, pixel_v)
        rospy.loginfo(f"ACTION: Grasping at X={target_pos[0]:.3f}, Y={target_pos[1]:.3f}, Z={target_pos[2]:.3f}")
        rospy.sleep(1.0)
        success = self._check_success()
        return 1.0 if success else 0.0

    def get_observation(self):
        if self.latest_pcd_combined is None: return None
        pcd = copy.deepcopy(self.latest_pcd_combined)

        # Crop
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=[ROI_X[0], ROI_Y[0], ROI_Z[0]], max_bound=[ROI_X[1], ROI_Y[1], ROI_Z[1]])
        pcd = pcd.crop(bbox)
        if len(pcd.points) == 0: return None
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step2)

        # Voxel
        pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step3)

        # Project
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        u = ((xyz[:, 1] - ROI_Y[0]) / GRID_RES).astype(int) 
        v = ((xyz[:, 0] - ROI_X[0]) / GRID_RES).astype(int) 
        u = np.clip(u, 0, IMG_H - 1)
        v = np.clip(v, 0, IMG_W - 1)

        tensor_map = np.zeros((IMG_H, IMG_W, 3), dtype=np.float32)
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        tensor_map[u, v] = rgb[sort_idx]

        if PUBLISH_DEBUG:
            img_uint8 = (tensor_map * 255).astype(np.uint8)
            msg = ros_numpy.msgify(Image, img_uint8, encoding='rgb8')
            # --- FIX: ADD HEADER INFO FOR RVIZ ---
            msg.header.frame_id = "base"
            msg.header.stamp = rospy.Time.now()
            self.pub_tensor.publish(msg)

        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def publish_o3d(self, pcd, publisher):
        """Publishes PointCloud2 with Correct RGB Encoding"""
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        if len(points) == 0: return

        # 1. Float 0..1 -> Int 0..255
        r = (colors[:, 0] * 255).astype(np.uint32)
        g = (colors[:, 1] * 255).astype(np.uint32)
        b = (colors[:, 2] * 255).astype(np.uint32)
        
        # --- FIX: BGR PACKING (Robot was blue, now it's red) ---
        rgb_uint32 = (b << 16) | (g << 8) | r 
        rgb_float = rgb_uint32.view(np.float32)

        data = np.zeros(len(points), dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
        data['x'], data['y'], data['z'], data['rgb'] = points[:,0], points[:,1], points[:,2], rgb_float
        
        msg = ros_numpy.msgify(PointCloud2, data)
        # --- FIX: ADD HEADER ---
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)

# ==============================================================================
# MAIN: OPENCV VISUALIZATION
# ==============================================================================
# MAIN: INTERACTIVE TEST MODE
# ==============================================================================
if __name__ == "__main__":
    import cv2
    env = GazeboEnv()
    
    rospy.loginfo("--- RUNNING IN INTERACTIVE DEBUG MODE ---")
    rospy.loginfo("Controls:")
    rospy.loginfo("  [r] -> Reset Spoon Position")
    rospy.loginfo("  [c] -> Check Success (Is it lifted?)")
    rospy.loginfo("  [q] -> Quit")

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # 1. Get Observation & Visualize
        obs_tensor = env.get_observation()
        
        if obs_tensor is not None:
            img_np = obs_tensor.permute(1, 2, 0).numpy()
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_large = cv2.resize(img_bgr, (512, 512), interpolation=cv2.INTER_NEAREST)
            
            # Overlay status text on image
            cv2.putText(img_large, "Press 'r' to Reset, 'c' to Check", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Dual Camera Input", img_large)

        # 2. Handle Keyboard Inputs
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        
        elif key == ord('r'):
            rospy.loginfo("Testing Reset...")
            env.reset()
            rospy.loginfo("Reset Complete.")
            
        elif key == ord('c'):
            rospy.loginfo("Checking Success...")
            is_picked = env._check_success()
            if is_picked:
                rospy.loginfo("RESULT: SUCCESS (Spoon is in the air!)")
            else:
                rospy.logwarn("RESULT: FAIL (Spoon is on the table)")

        rate.sleep()
        
    cv2.destroyAllWindows()