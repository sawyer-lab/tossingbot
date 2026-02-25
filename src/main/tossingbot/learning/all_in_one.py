#!/usr/bin/env python3.8
import sys
import os
import cv2
import time
import pickle
import numpy as np
import random

# --- PYTHON 3 COMPATIBILITY ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import rospkg
import torch
import torch.optim as optim
import torch.nn as nn
import torchvision.transforms.functional as TF

# --- ROS REPLACEMENT MESSAGES ---
from sawyer_robot.geometry import Point, Quaternion, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

# --- MODULAR IMPORTS ---
from tossingbot import config
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor
from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.hardware.gripper import GripperInterface
from sawyer_motion_planner import CasadiKinematics, CasadiPlanner, RotationPrimitive
from network import TossingBot_Modular
from buffer import PrioritizedReplayBuffer
from utils import RotationTransformer

# --- CONFIGURATION ---
LEARNING_RATE = 1e-4
MOMENTUM = 0.9
WEIGHT_DECAY = 2e-5
BATCH_SIZE = 8
BUFFER_CAPACITY = 2000
NUM_ROTATIONS = 4    
TRAIN_INTERVAL = 1    
GRADIENT_STEPS = 1    
SAVE_INTERVAL = 50    
SAVE_PATH = "tossingbot_auto.pth"

# Exploration
EXPLORE_START = 0.5   
EXPLORE_END = 0.1     
EXPLORE_STEPS = 15000

# Inhibition Settings
INHIBITION_RADIUS = 10 # Pixels around a failure to ignore

# ==============================================================================
# 1. SIMULATION INTERFACE
# ==============================================================================
class SimInterface:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        self.object_names = [f"banana_{i}" for i in range(5)] 
        self.anchor_poses = {} 

    def spawn_new_problem(self):
        self.anchor_poses = {} 
        MARGIN = 0.05
        min_x = config.ROI_X[0] + MARGIN
        max_x = config.ROI_X[1] - MARGIN
        min_y = config.ROI_Y[0] + MARGIN
        max_y = config.ROI_Y[1] - MARGIN

        for obj_name in self.object_names:
            q_list = RotationPrimitive.get_random_flat_quaternion()
            rand_x = np.random.uniform(min_x, max_x)
            rand_y = np.random.uniform(min_y, max_y)
            
            self.anchor_poses[obj_name] = {
                'pos': Point(rand_x, rand_y, 0.78),
                'ori': Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])
            }
        self._apply_anchor()

    def reset_to_anchor(self):
        if self.anchor_poses: self._apply_anchor()
        else: self.spawn_new_problem()

    def _apply_anchor(self):
        for obj_name, pose_data in self.anchor_poses.items():
            msg = ModelState()
            msg.model_name = obj_name
            msg.reference_frame = "world"
            msg.pose.position = pose_data['pos']
            msg.pose.orientation = pose_data['ori']
            msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
            msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = 0
            try: self.set_state_srv(msg)
            except rospy.ServiceException: pass
        rospy.sleep(0.5)

    def check_success(self):
        for obj_name in self.object_names:
            try:
                resp = self.get_state_srv(obj_name, "world")
                if resp.pose.position.z > 0.90:
                    rospy.loginfo(f"SUCCESS: Lifted {obj_name}!")
                    return True
            except rospy.ServiceException: pass
        return False

# ==============================================================================
# 2. ROTATIONAL ENVIRONMENT
# ==============================================================================
class RotationalEnv:
    def __init__(self, num_rotations=4):
        if rospy.get_node_uri() is None: rospy.init_node('rotational_env_node')

        self.camera = RosCamera(
            left_topic="/rgbd_camera_left/depth/points",
            right_topic="/rgbd_camera_right/depth/points"
        )
        self.vision = VisionProcessor()

        self.kinematics = CasadiKinematics(config.SAWYER_PNEUMATIC_URDF, "base", "right_gripper_tip")
        self.planner = CasadiPlanner(self.kinematics)

        self.robot = SawyerInterface()
        self.gripper = GripperInterface()
        self.sim = SimInterface()
        self.rot_helper = RotationPrimitive(num_rotations=num_rotations)
        self.NEUTRAL_Q = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0]

        # STEP TRACKING (As originally requested)
        self.max_steps = 50
        self.current_steps = 0

        rospy.loginfo("Waiting for Camera...")
        while self.camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def get_observation(self):
        pts, cols = self.camera.get_latest_cloud()
        if pts is None: return None
        return self.vision.process(pts, cols)

    def execute_plan(self, plan_data):
        if plan_data is None: return False
        stream = []
        for p in plan_data:
            stream.append(RobotCommand(
                position=p['position'],
                velocity=p['velocity'],
                acceleration=p['acceleration']
            ))
        return self.robot.execute_stream(stream, ControlMode.TRAJECTORY)

    def force_neutral(self):
        self.robot.move_to_joint_positions(self.NEUTRAL_Q)
        self.gripper.open()
        rospy.sleep(0.5)

    def reset_episode(self, previous_success=False, first_run=False):
        """
        Handles the 50-step limit logic.
        Returns: Observation + boolean (True if NEW SCENE, False if RETRY)
        """
        self.gripper.open()
        self.current_steps += 1
        
        is_new_scene = False

        if first_run:
            rospy.loginfo("INITIALIZING FIRST EPISODE...")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            is_new_scene = True
            
        elif previous_success:
            rospy.loginfo("GRASP SUCCESS! Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            is_new_scene = True
            
        elif self.current_steps >= self.max_steps:
            rospy.loginfo(f"MAX STEPS ({self.max_steps}) REACHED. Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            is_new_scene = True
            
        else:
            # RETRY SAME SCENE
            self.sim.reset_to_anchor()
            is_new_scene = False
            
        rospy.sleep(0.2)
        return self.get_observation(), is_new_scene

    def step(self, u, v, rot_idx):
        target_pos = self.vision.pixel_to_world(u, v)
        if target_pos[2] < 0.0: target_pos[2] = 0.0
        if np.linalg.norm(target_pos[:2]) > 0.90: return 0.0

        quat_msg = self.rot_helper.get_quaternion(rot_idx)
        target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        hover_pos = target_pos.copy(); hover_pos[2] += 0.20 
        grasp_pos = target_pos.copy()
        q_curr = self.robot.get_joint_positions()

        # 1. Hover
        ik_hover = self.planner.compute_inverse_kinematics(q_curr, hover_pos, target_quat)
        if ik_hover is None: return 0.0
        self.robot.move_to_joint_positions(ik_hover, timeout=2.0)

        # 2. Descend
        q_curr = self.robot.get_joint_positions()
        path_down = self.planner.plan_cartesian(q_curr, grasp_pos, target_quat, 2.0, check_floor=False)
        if path_down is None: return 0.0
        if not self.execute_plan(path_down): return 0.0

        # 3. Grasp
        self.gripper.close()
        rospy.sleep(0.2)
        
        # 4. Lift
        q_curr = self.robot.get_joint_positions()
        path_up = self.planner.plan_cartesian(q_curr, hover_pos, target_quat, 2.0, check_floor=False)
        if path_up is not None: self.execute_plan(path_up)

        return 1.0 if self.sim.check_success() else 0.0

# ==============================================================================
# 3. BRAIN NODE
# ==============================================================================
class TossingNode:
    def __init__(self):
        rospy.init_node('tossingbot_brain')
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.env = RotationalEnv(num_rotations=NUM_ROTATIONS)
        self.transformer = RotationTransformer(device=self.device)
        self.buffer = PrioritizedReplayBuffer(capacity=BUFFER_CAPACITY)

        self.model = TossingBot_Modular(input_channels=4).to(self.device)
        self.optimizer = optim.SGD(self.model.parameters(), lr=LEARNING_RATE, momentum=MOMENTUM, weight_decay=WEIGHT_DECAY)
        self.loss_fn = nn.BCEWithLogitsLoss(reduction='none') 

        rp = rospkg.RosPack()
        self.save_dir = os.path.join(rp.get_path('tossingbot_system'), "weights")
        self.weights_path = os.path.join(self.save_dir, SAVE_PATH)
        
        self.step_count = 0
        
        # --- SHORT TERM MEMORY FOR FAILURE INHIBITION ---
        # Stores tuples of (rot_idx, u, v) that failed in the current scene
        self.failed_attempts_in_scene = [] 

        self.load_snapshot()
        cv2.namedWindow("Auto Brain", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Auto Brain", 1200, 800)

    def get_epsilon(self):
        if self.step_count >= EXPLORE_STEPS: return EXPLORE_END
        fraction = float(self.step_count) / float(EXPLORE_STEPS)
        return EXPLORE_START - fraction * (EXPLORE_START - EXPLORE_END)

    def apply_inhibition(self, full_vol):
        """
        Masks out areas around previously failed attempts so the argmax
        chooses the next best peak.
        """
        if not self.failed_attempts_in_scene:
            return full_vol

        # Clone so we don't modify the original tensor used for backprop (if we were training on it)
        masked_vol = full_vol.clone()
        B, R, H, W = masked_vol.shape
        
        # We fill a square region with a very small number (-1e9)
        for (f_rot, f_u, f_v) in self.failed_attempts_in_scene:
            u_min = max(0, f_u - INHIBITION_RADIUS)
            u_max = min(H, f_u + INHIBITION_RADIUS)
            v_min = max(0, f_v - INHIBITION_RADIUS)
            v_max = min(W, f_v + INHIBITION_RADIUS)
            
            # Mask the specific rotation channel
            masked_vol[0, f_rot, u_min:u_max, v_min:v_max] = -1e9
            
        return masked_vol

    def run(self):
        rospy.loginfo("STARTING SELF-SUPERVISED LOOP...")
        self.env.force_neutral()
        
        # Initial Reset
        state_tensor, _ = self.env.reset_episode(first_run=True)
        self.failed_attempts_in_scene = []
        
        while not rospy.is_shutdown():
            if state_tensor is None:
                 state_tensor = self.env.get_observation()
                 if state_tensor is None: rospy.sleep(0.1); continue

            state_gpu = state_tensor.to(self.device)

            # --- 1. FORWARD PASS ---
            with torch.no_grad():
                full_vol, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)
                
            # --- 2. APPLY INHIBITION (Don't be stubborn!) ---
            masked_vol = self.apply_inhibition(full_vol)

            # --- 3. SELECT ACTION ---
            epsilon = self.get_epsilon()
            H, W = full_vol.shape[2:] # B, C, H, W
            
            if random.random() < epsilon:
                action_type = "RND"
                rot_idx = random.randint(0, NUM_ROTATIONS - 1)
                margin = 15
                u_rot = random.randint(margin, H - margin - 1) 
                v_rot = random.randint(margin, W - margin - 1)
                conf = 0.0
            else:
                action_type = "NET"
                # Argmax on MASKED volume
                flat_idx = torch.argmax(masked_vol).item()
                rot_idx = flat_idx // (H * W)
                rem = flat_idx % (H * W)
                u_rot = rem // W
                v_rot = rem % W
                
                # Get confidence from ORIGINAL volume (for display)
                conf = torch.sigmoid(full_vol[0, rot_idx, u_rot, v_rot]).item()

            angle_deg = self.env.rot_helper.get_angle(rot_idx)
            
            # --- 4. VISUALIZE ---
            self.visualize_dashboard(state_gpu, rotated_inputs, raw_heatmaps, rot_idx, u_rot, v_rot, conf)

            # --- 5. EXECUTE ---
            u_world, v_world = self.transformer.rotate_pixel(u_rot, v_rot, angle_deg, H, W, to_gripper_frame=False)
            
            print("-" * 50)
            print(f"[STEP {self.step_count}] Mode: {action_type} (Eps: {epsilon:.2f})")
            print(f"   >>> Rot: {rot_idx} ({angle_deg:.1f}°) | Pixel: ({u_rot}, {v_rot})")
            if action_type == "NET" and len(self.failed_attempts_in_scene) > 0:
                print(f"   >>> (Inhibiting {len(self.failed_attempts_in_scene)} previous failures)")

            reward = self.env.step(u_world, v_world, rot_idx)
            msg = "SUCCESS" if reward > 0.5 else "FAIL"
            print(f"   >>> Result: {msg} (Rew: {reward})")

            # --- 6. STORE & TRAIN ---
            self.buffer.push(state_tensor.cpu(), u_rot, v_rot, rot_idx, reward)
            
            if self.step_count % TRAIN_INTERVAL == 0 and len(self.buffer) > BATCH_SIZE:
                self.train_burst()
            if self.step_count % SAVE_INTERVAL == 0 and self.step_count > 0:
                self.save_snapshot()

            self.step_count += 1
            
            # --- 7. HANDLE FAILURES / RESETS ---
            success = (reward > 0.5)
            
            # Get next state and check if the scene changed
            state_tensor, is_new_scene = self.env.reset_episode(previous_success=success)
            
            if is_new_scene:
                # Clear short-term memory (new objects = new grasp points)
                self.failed_attempts_in_scene = []
            elif not success:
                # Same scene, but we failed. Remember this spot so we don't try it again.
                self.failed_attempts_in_scene.append((rot_idx, u_rot, v_rot))

    def forward_multi_view(self, state_tensor):
        batch_rotated = []
        c, h, w = state_tensor.shape
        diag_len = int(np.sqrt(h**2 + w**2))
        pad_x = (diag_len - w) // 2
        pad_y = (diag_len - h) // 2
        padded_state = TF.pad(state_tensor, [pad_x, pad_y, pad_x, pad_y], fill=0)

        for i in range(NUM_ROTATIONS):
            angle = -self.env.rot_helper.get_angle(i)
            rot_padded = TF.rotate(padded_state, angle, interpolation=TF.InterpolationMode.BILINEAR)
            rot_img = TF.center_crop(rot_padded, [h, w])
            batch_rotated.append(rot_img)
        
        rotated_inputs_vol = torch.stack(batch_rotated)
        batch_out = self.model(rotated_inputs_vol)
        return batch_out.squeeze(1).unsqueeze(0), rotated_inputs_vol, batch_out

    def train_burst(self):
        total_loss = 0.0
        for _ in range(GRADIENT_STEPS):
            batch, indices, weights = self.buffer.sample(BATCH_SIZE)
            b_states = [x[0] for x in batch]
            b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]
            b_rot = [x[3] for x in batch]
            b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)
            weights = torch.tensor(weights, dtype=torch.float32).to(self.device).unsqueeze(1)

            rotated_imgs, valid_pixels_u, valid_pixels_v = [], [], []
            for i in range(len(batch)):
                st = b_states[i].to(self.device)
                rot_idx = b_rot[i]
                angle = self.env.rot_helper.get_angle(rot_idx)
                rot_img = self.transformer.to_gripper_frame(st, -angle)
                rotated_imgs.append(rot_img)
                valid_pixels_u.append(b_u[i])
                valid_pixels_v.append(b_v[i])

            tensor_input = torch.stack(rotated_imgs)
            self.optimizer.zero_grad()
            logits = self.model(tensor_input)
            
            pred_vals = []
            for i in range(len(batch)):
                u, v = valid_pixels_u[i], valid_pixels_v[i]
                u = min(max(u, 0), logits.shape[2]-1)
                v = min(max(v, 0), logits.shape[3]-1)
                pred_vals.append(logits[i, 0, u, v])
            
            pred_vals = torch.stack(pred_vals).unsqueeze(1)
            loss = self.loss_fn(pred_vals, b_rew) * weights
            loss = loss.mean()
            loss.backward()
            self.optimizer.step()
            errors = torch.abs(pred_vals - b_rew).detach().cpu().numpy()
            self.buffer.update_priorities(indices, errors)
            total_loss += loss.item()
        self.current_loss = total_loss / GRADIENT_STEPS

    def _tensor_to_cv(self, tensor_img):
        img = tensor_img[:3, :, :].permute(1, 2, 0).detach().cpu().numpy()
        img = np.clip(img * 255, 0, 255).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def visualize_dashboard(self, state, rotated_inputs, raw_heatmaps, chosen_rot, u_rot, v_rot, conf):
        pairs = []
        indices_to_show = np.linspace(0, NUM_ROTATIONS-1, 4, dtype=int)
        if chosen_rot not in indices_to_show:
            indices_to_show[-1] = chosen_rot; indices_to_show.sort()

        for i in indices_to_show:
            input_cv = self._tensor_to_cv(rotated_inputs[i])
            h = raw_heatmaps[0, i].detach().cpu().numpy()
            h = 1.0 / (1.0 + np.exp(-h)) 
            h_img = (h * 255).astype(np.uint8)
            heatmap_cv = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)
            
            # Draw previously failed spots as X
            if i == chosen_rot:
                for (fr, fu, fv) in self.failed_attempts_in_scene:
                    if fr == i:
                        cv2.drawMarker(input_cv, (fv, fu), (0,0,255), markerType=cv2.MARKER_CROSS, thickness=1)

            if i == chosen_rot:
                cv2.circle(input_cv, (v_rot, u_rot), 4, (0, 0, 255), -1)
                cv2.rectangle(input_cv, (0,0), (input_cv.shape[1], input_cv.shape[0]), (0,255,0), 2)
            
            pair = np.hstack([input_cv, heatmap_cv])
            pairs.append(pair)

        top_row = np.hstack(pairs)
        world_rgb = self._tensor_to_cv(state)
        H, W, _ = world_rgb.shape
        angle = self.env.rot_helper.get_angle(chosen_rot)
        u_world, v_world = self.transformer.rotate_pixel(u_rot, v_rot, angle, H, W, to_gripper_frame=False)
        cv2.circle(world_rgb, (v_world, u_world), 5, (0, 255, 0), -1)
        angle_rad = np.deg2rad(-angle)
        end_x = int(v_world + 40 * np.cos(angle_rad))
        end_y = int(u_world + 40 * np.sin(angle_rad))
        cv2.arrowedLine(world_rgb, (v_world, u_world), (end_x, end_y), (0, 0, 255), 2)

        scale = top_row.shape[1] / world_rgb.shape[1]
        new_h = int(world_rgb.shape[0] * scale)
        bottom_row = cv2.resize(world_rgb, (top_row.shape[1], new_h))
        final_img = np.vstack([top_row, bottom_row])
        cv2.imshow("Auto Brain", final_img)
        cv2.waitKey(1)

    def save_snapshot(self):
        if not os.path.exists(self.save_dir): os.makedirs(self.save_dir)
        checkpoint = {'model_state': self.model.state_dict(), 'optimizer_state': self.optimizer.state_dict(), 'step_count': self.step_count}
        torch.save(checkpoint, self.weights_path)
        try:
            with open(self.weights_path.replace(".pth", "_buffer.pkl"), 'wb') as f: pickle.dump(self.buffer, f)
            rospy.loginfo(f"SAVED SNAPSHOT: Step {self.step_count}")
        except: pass

    def load_snapshot(self):
        if os.path.exists(self.weights_path):
            try:
                ckpt = torch.load(self.weights_path, map_location=self.device)
                self.model.load_state_dict(ckpt['model_state'])
                self.optimizer.load_state_dict(ckpt['optimizer_state'])
                self.step_count = ckpt['step_count']
            except: pass
        buf_path = self.weights_path.replace(".pth", "_buffer.pkl")
        if os.path.exists(buf_path):
            try:
                with open(buf_path, 'rb') as f: self.buffer = pickle.load(f)
            except: pass

if __name__ == '__main__':
    TossingNode().run()