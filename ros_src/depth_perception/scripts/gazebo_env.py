#!/usr/bin/env python3.8
import sys

# 1. PYTHON 3 COMPATIBILITY HACKS (Crucial for ROS Melodic)
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import open3d as o3d
import torch
import message_filters # <--- NEW: For syncing two cameras
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

        rospy.loginfo("Initializing Gazebo Environment (Dual Camera Merger)...")

        # 1. ROS Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # 2. Perception Input (Dual Camera Sync)
        self.latest_pcd_combined = None # This will hold the merged Open3D cloud

        # --- 1. SERVICE PROXIES (Teleporting & Checking) ---
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # Subscribe to both topics
        left_sub = message_filters.Subscriber("/rgbd_camera_left/depth/points", PointCloud2)
        right_sub = message_filters.Subscriber("/rgbd_camera_right/depth/points", PointCloud2)
        
        # ApproximateTimeSynchronizer allows slight time differences (slop=0.1s)
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

    def _reset_spoon(self):
        """Teleports the spoon back to the center of the table"""
        state_msg = ModelState()
        state_msg.model_name = OBJECT_NAME
        state_msg.reference_frame = "world"
        
        # Set Position (Center of ROI, slightly dropped)
        # Randomize slightly so it's not ALWAYS the exact same millimeter
        rand_x = np.random.uniform(ROI_X[0]+0.1, ROI_X[1]-0.1)
        rand_y = np.random.uniform(ROI_Y[0]+0.1, ROI_Y[1]-0.1)
        
        state_msg.pose.position = Point(rand_x, rand_y, 0.1)
        state_msg.pose.orientation = Quaternion(0, 0, 0, 1) # Flat

        try:
            self.set_state_srv(state_msg)
            rospy.sleep(0.5) # Wait for it to fall and settle
        except rospy.ServiceException as e:
            rospy.logwarn(f"Reset failed: {e}")

    def _check_success(self):
        """Returns True if spoon is lifted > 20cm above table"""
        try:
            # Ask Gazebo where the spoon is relative to the world
            resp = self.get_state_srv(OBJECT_NAME, "world")
            
            if not resp.success:
                rospy.logwarn("Could not get object state!")
                return False

            # Check Z Height
            # Assuming Table is at Z=0.0 (or slightly below). 
            # If Z > 0.2, the robot is holding it up.
            z_height = resp.pose.position.z
            
            if z_height > 0.20: 
                rospy.loginfo(f"SUCCESS! Object lifted to {z_height:.2f}m")
                return True
            else:
                return False
                
        except rospy.ServiceException as e:
            rospy.logwarn(f"Check success failed: {e}")
            return False

    def _ros_to_o3d(self, msg):
        """Helper: Converts ROS PointCloud2 -> Open3D PointCloud"""
        try:
            pc_np = ros_numpy.numpify(msg).reshape(-1)
        except Exception: return None

        points = np.zeros((pc_np.shape[0], 3), dtype=np.float64)
        points[:,0], points[:,1], points[:,2] = pc_np['x'], pc_np['y'], pc_np['z']
        
        # RGB Unpacking
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
        """Helper: Transforms Open3D cloud to 'base' frame using TF"""
        try:
            trans = self.tf_buffer.lookup_transform("base", frame_id, rospy.Time(0), rospy.Duration(0.1))
            t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
            R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            T = np.eye(4); T[:3,:3]=R; T[:3,3]=t
            pcd.transform(T)
            return pcd
        except Exception as e:
            rospy.logwarn(f"TF Error for {frame_id}: {e}")
            return None

    def _cb_clouds_merged(self, msg_left, msg_right):
        """
        Callback triggered only when BOTH cameras have data.
        1. Convert Left -> O3D -> Transform to Base
        2. Convert Right -> O3D -> Transform to Base
        3. Add them together
        """
        # A. Process Left
        pcd_left = self._ros_to_o3d(msg_left)
        pcd_left = self._transform_cloud(pcd_left, msg_left.header.frame_id)
        
        # B. Process Right
        pcd_right = self._ros_to_o3d(msg_right)
        pcd_right = self._transform_cloud(pcd_right, msg_right.header.frame_id)

        if pcd_left is None or pcd_right is None: return

        # C. Merge
        # In Open3D, merging is just addition
        pcd_combined = pcd_left + pcd_right
        
        self.latest_pcd_combined = pcd_combined

        # (Optional) Publish the raw merged cloud to visualize alignment
        if PUBLISH_DEBUG:
            self.publish_o3d(self.latest_pcd_combined, self.pub_step1)

    def reset(self):
        self._reset_spoon() # <--- Calls our new reset
        return self.get_observation()

    def step(self, pixel_u, pixel_v):
        # 1. Calculate World Target
        world_x = ROI_X[0] + (pixel_v * GRID_RES)
        world_y = ROI_Y[0] + (pixel_u * GRID_RES)
        
        rospy.loginfo(f"ACTION: Grasping at X={world_x:.3f}, Y={world_y:.3f}")

        # 2. EXECUTE GRASP (MoveIt Logic goes here later)
        # For now, we simulate a grasp by sleeping
        rospy.sleep(1.0) 
        
        # 3. LIFT (MoveIt Logic goes here later)
        # move_group.set_target(up) ...
        
        # 4. CHECK SUCCESS
        success = self._check_success() # <--- Calls our new check
        
        reward = 1.0 if success else 0.0
        return reward

    def get_observation(self):
        """
        Uses self.latest_pcd_combined which is already Transformed & Merged.
        """
        if self.latest_pcd_combined is None: return None
        
        # Use a copy so we don't mess up the buffer while new data comes in
        pcd = copy.deepcopy(self.latest_pcd_combined)

        # --- STEP 2: Crop to Workspace ---
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[ROI_X[0], ROI_Y[0], ROI_Z[0]], 
            max_bound=[ROI_X[1], ROI_Y[1], ROI_Z[1]]
        )
        pcd = pcd.crop(bbox)
        if len(pcd.points) == 0: return None
        
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step2)

        # --- STEP 3: Voxel/Clean (CRITICAL FOR MERGING) ---
        # Voxel grid merges overlapping points from Left/Right cameras
        pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step3)

        # --- STEP 4: Project to Tensor ---
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        u = ((xyz[:, 1] - ROI_Y[0]) / GRID_RES).astype(int) 
        v = ((xyz[:, 0] - ROI_X[0]) / GRID_RES).astype(int) 
        u = np.clip(u, 0, IMG_H - 1)
        v = np.clip(v, 0, IMG_W - 1)

        tensor_map = np.zeros((IMG_H, IMG_W, 3), dtype=np.float32)
        sort_idx = np.argsort(xyz[:, 2]) # Sort by Z
        u, v = u[sort_idx], v[sort_idx]
        tensor_map[u, v] = rgb[sort_idx] # Update map

        if PUBLISH_DEBUG:
            img_uint8 = (tensor_map * 255).astype(np.uint8)
            msg = ros_numpy.msgify(Image, img_uint8, encoding='rgb8')
            msg.header.frame_id = "base"
            msg.header.stamp = rospy.Time.now()
            self.pub_tensor.publish(msg)

        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def publish_o3d(self, pcd, publisher):
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        if len(points) == 0: return

        r = (colors[:, 0] * 255).astype(np.uint32)
        g = (colors[:, 1] * 255).astype(np.uint32)
        b = (colors[:, 2] * 255).astype(np.uint32)
        rgb_uint32 = (b << 16) | (g << 8) | r 
        rgb_float = rgb_uint32.view(np.float32)

        data = np.zeros(len(points), dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
        data['x'], data['y'], data['z'], data['rgb'] = points[:,0], points[:,1], points[:,2], rgb_float
        
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)

import copy # Added for deepcopy

if __name__ == "__main__":
    import cv2
    env = GazeboEnv()
    rospy.loginfo("--- RUNNING IN OPENCV DEBUG MODE ---")
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        obs_tensor = env.get_observation()
        if obs_tensor is not None:
            img_np = obs_tensor.permute(1, 2, 0).numpy()
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_large = cv2.resize(img_bgr, (512, 512), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Dual Camera Input", img_large)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
        rate.sleep()
    cv2.destroyAllWindows()