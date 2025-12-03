#!/usr/bin/env python3.8
import sys

sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import open3d as o3d
import torch
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# Bounds (Robot Frame)
ROI_X = [0.3, 0.8]    # Forward/Back
ROI_Y = [-0.25, 0.25] # Left/Right
ROI_Z = [-0.1, 0.5]   # Table Height -> Up

# Resolution
VOXEL_SIZE = 0.005
GRID_RES = 0.005 

# Calculated Dimensions
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Debug Switches
PUBLISH_DEBUG = True
# ==============================================================================

class GazeboEnv:
    def __init__(self):
        if rospy.get_node_uri() is None:
            rospy.init_node('gazebo_env_node')

        rospy.loginfo("Initializing Gazebo Environment with Perception Pipeline...")

        # 1. ROS Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # 2. Perception Input
        self.latest_cloud = None
        # CHECK YOUR TOPIC NAME: Use 'rostopic list' to confirm
        rospy.Subscriber("/rgbd_camera/depth/points", PointCloud2, self._cb_cloud)

        # 3. Debug Publishers (The logic from PerceptionDebugger)
        if PUBLISH_DEBUG:
            self.pub_step1 = rospy.Publisher("/debug/1_transformed", PointCloud2, queue_size=1)
            self.pub_step2 = rospy.Publisher("/debug/2_cropped", PointCloud2, queue_size=1)
            self.pub_step3 = rospy.Publisher("/debug/3_clean", PointCloud2, queue_size=1)
            self.pub_tensor = rospy.Publisher("/debug/4_tensor_img", Image, queue_size=1)

        # 4. Robot Control (Placeholders)
        # self.arm = ... 
        
        rospy.loginfo("Env Ready. Waiting for PointCloud...")
        while self.latest_cloud is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("PointCloud Received!")

    def _cb_cloud(self, msg):
        """Buffer the latest ROS message"""
        self.latest_cloud = msg

    def reset(self):
        """Resets Gazebo and returns first observation"""
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_srv()
        rospy.sleep(1.0) # Wait for dust to settle
        return self.get_observation()

    def step(self, pixel_u, pixel_v):
        """Execute Action -> Return Reward"""
        # Convert Pixel to World
        world_x = ROI_X[0] + (pixel_v * GRID_RES)
        world_y = ROI_Y[0] + (pixel_u * GRID_RES)
        
        rospy.loginfo(f"ACTION: Grasp at X={world_x:.3f}, Y={world_y:.3f}")
        
        # TODO: Add your MoveIt/Gripper logic here
        
        rospy.sleep(0.5)
        reward = 0.0 # Dummy reward
        return reward

    def get_observation(self):
        """
        The Full Perception Pipeline.
        Returns: Torch Tensor (3, H, W)
        Side Effect: Publishes to Rviz if PUBLISH_DEBUG is True
        """
        if self.latest_cloud is None: return None

        # --- STEP 0: Parse ROS Message ---
        try:
            pc_np = ros_numpy.numpify(self.latest_cloud).reshape(-1)
        except Exception: return None

        points = np.zeros((pc_np.shape[0], 3), dtype=np.float64)
        points[:,0], points[:,1], points[:,2] = pc_np['x'], pc_np['y'], pc_np['z']
        
        # Handle RGB Packing (Bit shifting fix)
        rgb_f32 = pc_np['rgb']
        rgb_u32 = rgb_f32.view(np.uint32)
        r = ((rgb_u32 >> 16) & 0xFF) / 255.0
        g = ((rgb_u32 >> 8) & 0xFF) / 255.0
        b = (rgb_u32 & 0xFF) / 255.0
        colors = np.stack([r, g, b], axis=-1)

        # Open3D Object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # --- STEP 1: Transform Camera -> Base ---
        try:
            trans = self.tf_buffer.lookup_transform("base", self.latest_cloud.header.frame_id, rospy.Time(0))
            t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
            R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
            T = np.eye(4); T[:3,:3]=R; T[:3,3]=t
            pcd.transform(T)
        except Exception as e:
            rospy.logwarn(f"TF Error: {e}")
            return None

        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step1)

        # --- STEP 2: Crop to Workspace ---
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[ROI_X[0], ROI_Y[0], ROI_Z[0]], 
            max_bound=[ROI_X[1], ROI_Y[1], ROI_Z[1]]
        )
        pcd = pcd.crop(bbox)
        if len(pcd.points) == 0: return None
        
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step2)

        # --- STEP 3: Voxel/Clean ---
        pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        if PUBLISH_DEBUG: self.publish_o3d(pcd, self.pub_step3)

        # --- STEP 4: Project to Tensor (Ortho Projection) ---
        xyz = np.asarray(pcd.points)
        rgb = np.asarray(pcd.colors)

        # Map X,Y to U,V
        u = ((xyz[:, 1] - ROI_Y[0]) / GRID_RES).astype(int) # Y -> Rows (u)
        v = ((xyz[:, 0] - ROI_X[0]) / GRID_RES).astype(int) # X -> Cols (v)
        
        u = np.clip(u, 0, IMG_H - 1)
        v = np.clip(v, 0, IMG_W - 1)

        # Create Maps
        tensor_map = np.zeros((IMG_H, IMG_W, 3), dtype=np.float32)
        
        # Z-Sort (Simple Painter's Algorithm)
        sort_idx = np.argsort(xyz[:, 2])
        u, v = u[sort_idx], v[sort_idx]
        rgb = rgb[sort_idx]
        
        tensor_map[u, v] = rgb # Only storing RGB for input

        # Publish Debug Image
        if PUBLISH_DEBUG:
            img_uint8 = (tensor_map * 255).astype(np.uint8)
            msg = ros_numpy.msgify(Image, img_uint8, encoding='rgb8')
            msg.header.frame_id = "base"
            msg.header.stamp = rospy.Time.now()
            self.pub_tensor.publish(msg)

        # Return PyTorch Tensor (Channels First: 3, H, W)
        return torch.from_numpy(tensor_map).permute(2, 0, 1)

    def publish_o3d(self, pcd, publisher):
        """Helper to publish Open3D objects to RViz with FIXED COLORS"""
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        if len(points) == 0: return

        # Bit shifting (BGR Swap Logic included)
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

# ==============================================================================
# MAIN: RUN THIS TO TEST PERCEPTION (Like the Debugger)
# ==============================================================================
# ==============================================================================
# MAIN: RUN THIS TO VISUALIZE (OPENCV WINDOW)
# ==============================================================================
if __name__ == "__main__":
    # Import cv2 only for debugging so it doesn't clutter the main class
    import cv2
    
    env = GazeboEnv()
    rospy.loginfo("--- RUNNING IN OPENCV DEBUG MODE ---")
    rospy.loginfo("Press 'q' in the window to quit.")

    rate = rospy.Rate(10) # 10 Hz refresh
    
    while not rospy.is_shutdown():
        # 1. Get the Tensor (3, H, W)
        obs_tensor = env.get_observation()

        if obs_tensor is not None:
            # 2. Convert PyTorch Tensor -> Numpy Image
            # Change shape from (Channels, Height, Width) -> (Height, Width, Channels)
            img_np = obs_tensor.permute(1, 2, 0).numpy()

            # 3. Handle Color Conversion
            # The tensor is RGB (Floats 0..1). OpenCV expects BGR.
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            
            # 4. Scale up! 
            # The network sees 128x128, but we want to see 500x500 to check details.
            # INTER_NEAREST keeps the pixels "blocky" so you can see exact grid alignment.
            img_large = cv2.resize(img_bgr, (512, 512), interpolation=cv2.INTER_NEAREST)

            # 5. Show it
            cv2.imshow("Neural Network Input", img_large)
            
            # Exit logic
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            rospy.loginfo_throttle(2.0, "Waiting for valid PointCloud data...")

        rate.sleep()

    cv2.destroyAllWindows()