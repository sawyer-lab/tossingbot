#!/usr/bin/env python3.8
import sys
import os

# 1. PYTHON 3 COMPATIBILITY HACKS
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros

import numpy as np


import cv2
import actionlib
# import h5py
from datetime import datetime


from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion

# Make sure your package is sourced
from grasping.msg import GraspAction, GraspGoal
from perception_module import PerceptionModule
from tossingbot import config

# ==============================================================================
# CONFIGURATION
# ==============================================================================

OBJECT_NAME = "banana"
PUBLISH_DEBUG = True
SAVE_DATA = False 
DATA_DIR = "/geometry2_ws/data_collection" 


class DataCollector:
    def __init__(self, save_dir):
        self.save_dir = save_dir
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        self.buffer = []
        self.save_interval = 100 # Flush to disk every 100 steps
        rospy.loginfo(f"DataCollector initialized at {save_dir}")

    def store(self, obs_tensor, action_uv, reward, success):
        """
        obs_tensor: torch.Tensor (3, H, W)
        action_uv: tuple (u, v)
        reward: float
        success: bool
        """
        if obs_tensor is None: return

        # Convert Torch -> Numpy (H, W, 3) for storage
        obs_np = obs_tensor.permute(1, 2, 0).cpu().numpy()
        
        transition = {
            'obs': obs_np,
            'action': np.array(action_uv, dtype=np.int32),
            'reward': float(reward),
            'success': int(success)
        }
        self.buffer.append(transition)

        if len(self.buffer) >= self.save_interval:
            self.flush()

    def flush(self):
        if not self.buffer: return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(self.save_dir, f"training_data_{timestamp}.h5")
        
        # Stack data
        obs_stack = np.stack([t['obs'] for t in self.buffer])
        act_stack = np.stack([t['action'] for t in self.buffer])
        rew_stack = np.array([t['reward'] for t in self.buffer])
        suc_stack = np.array([t['success'] for t in self.buffer])

        # Write HDF5
        with h5py.File(filename, 'w') as f:
            f.create_dataset('observations', data=obs_stack, compression="gzip")
            f.create_dataset('actions', data=act_stack)
            f.create_dataset('rewards', data=rew_stack)
            f.create_dataset('success', data=suc_stack)
            
        rospy.loginfo(f"[DataCollector] Saved {len(self.buffer)} samples to {filename}")
        self.buffer = []


class SimInterface:
    def __init__(self):
        # Service Proxies
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.reset_sim_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def reset_object(self):
        state_msg = ModelState()
        state_msg.model_name = OBJECT_NAME
        state_msg.reference_frame = "world"
        
        # Randomize Position
        rand_x = np.random.uniform(config.ROI_X[0]+0.1, config.ROI_X[1]-0.1)
        rand_y = np.random.uniform(config.ROI_Y[0]+0.1, config.ROI_Y[1]-0.1)
        
        # 1. Pose
        state_msg.pose.position = Point(rand_x, rand_y, 0.75) # Lower drop height
        state_msg.pose.orientation = Quaternion(0, 0, 0, 1)

        # 2. PHYSICS FIX: Explicitly zero velocities to stop spinning
        state_msg.twist.linear.x = 0; state_msg.twist.linear.y = 0; state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0; state_msg.twist.angular.y = 0; state_msg.twist.angular.z = 0

        try:
            self.set_state_srv(state_msg)
            # Short sleep to let physics settle (if running 10x speed, 0.2s is enough)
            rospy.sleep(0.2) 
        except rospy.ServiceException as e:
            rospy.logwarn(f"Reset failed: {e}")

    def check_success(self):
        try:
            resp = self.get_state_srv(OBJECT_NAME, "world")
            if not resp.success: return False
            # Check if Z is lifted > 0.90m
            return resp.pose.position.z > 0.90
        except rospy.ServiceException:
            return False


class GraspController:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('grasping_action', GraspAction)
        rospy.loginfo("Waiting for Grasping Server...")
        if self.client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.loginfo("Connected to Grasping Server.")
        else:
            rospy.logwarn("Grasping Server NOT detected.")

    def execute(self, x, y, z):
        goal = GraspGoal()
        goal.target_position = Point(x=x, y=y, z=z)
        goal.orientation_index = 0
        
        self.client.send_goal(goal)
        # Wait up to 20s (Sim time)
        finished = self.client.wait_for_result(timeout=rospy.Duration(20.0))
        
        if not finished:
            self.client.cancel_goal()
            return False
            
        return self.client.get_result().success





class GazeboEnv:
    def __init__(self):
        if rospy.get_node_uri() is None:
            rospy.init_node('gazebo_env_node')

        rospy.loginfo("Initializing Modular Gazebo Environment...")
        
        # Shared Resources
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize Sub-Modules
        self.perception = PerceptionModule(self.tf_buffer)
        self.sim = SimInterface()
        self.robot = GraspController()
        
        if SAVE_DATA:
            self.collector = DataCollector(DATA_DIR)

        # Wait for Vision
        while self.perception.latest_pcd_combined is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("System Ready!")

    def reset(self):
        self.sim.reset_object()
        return self.get_observation()

    def step(self, pixel_u, pixel_v):
        # 1. Perception: Pixel -> World
        target_pos = self.perception.pixel_to_world(pixel_u, pixel_v)
        rospy.loginfo(f"ACTION: {pixel_u}, {pixel_v} -> {target_pos}")

        # 2. Control: Move Robot
        motion_success = self.robot.execute(target_pos[0], target_pos[1], target_pos[2])
        
        if not motion_success:
            rospy.logwarn("Robot Motion Failed.")
            return 0.0

        # 3. Sim: Check Success
        success = self.sim.check_success()
        reward = 1.0 if success else 0.0
        
        return reward

    def get_observation(self):
        return self.perception.get_observation_tensor()

    def save_step(self, obs, u, v, reward):
        if SAVE_DATA:
            self.collector.store(obs, (u, v), reward, (reward > 0.5))



if __name__ == "__main__":
    env = GazeboEnv()
    
    rospy.loginfo("--- INTERACTIVE MODE WITH DATA COLLECTION ---")
    
    WINDOW_NAME = "Dual Camera Input"
    cv2.namedWindow(WINDOW_NAME)

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Map window coords -> grid coords
            scale_x = config.IMG_W / 512.0
            scale_y = config.IMG_H / 512.0
            real_v = int(x * scale_x)
            real_u = int(y * scale_y)
            
            # 1. Capture current observation BEFORE acting
            obs = env.get_observation()
            
            # 2. Execute
            reward = env.step(real_u, real_v)
            rospy.loginfo(f"Reward: {reward}")

            # 3. Save Data (HDF5)
            env.save_step(obs, real_u, real_v, reward)

    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Visual Loop
        obs_tensor = env.get_observation()
        if obs_tensor is not None:
            img_np = obs_tensor.permute(1, 2, 0).numpy()
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_large = cv2.resize(img_bgr, (512, 512), interpolation=cv2.INTER_NEAREST)
            cv2.putText(img_large, "Click to Grasp & Save | r: Reset", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, img_large)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('r'): env.reset()

        rate.sleep()
        
    # Flush remaining data on exit
    if SAVE_DATA:
        env.collector.flush()
    cv2.destroyAllWindows()