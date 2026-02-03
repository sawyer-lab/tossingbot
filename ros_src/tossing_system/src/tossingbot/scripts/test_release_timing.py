#!/usr/bin/env python3.8
import rospy
import numpy as np
import rospkg
import json
import os
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetModelState
from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot.environment.landing_sensor import LandingSensor
from tossingbot import config as cfg
from tossingbot.alignment import TargetAlignment

# --- CONFIGURATION ---
SUITE_NAME = "release_timing_test"
LOG_DIR = os.path.join(cfg.PACKAGE_ROOT, "logs", "timing_test")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"{SUITE_NAME}_{int(time.time())}.json")

TRIALS_PER_CONFIG = 3
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
ANGLE_DEG = 0.0 # Straight toss for timing test

# Constants
G = 9.806
TABLE_Z = 0.75

class TrajectoryRecorder:
    def __init__(self, object_name, gripper):
        self.object_name = object_name
        self.gripper = gripper
        self.history = []
        self.recording = False
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def start(self):
        self.history = []
        self.recording = True
        
    def stop(self):
        self.recording = False
        return self.history

    def record_step(self, cmd_active=False):
        if not self.recording: return
        try:
            # Object State
            resp = self.get_state(self.object_name, "world")
            
            # Gripper State
            grip_width = self.gripper.get_current_position()
            
            item = {
                "t": rospy.get_time(),
                "grip_width": grip_width,
                "cmd_active": cmd_active # True if release command has been sent
            }
            
            if resp.success:
                p = resp.pose.position
                v = resp.twist.linear
                item["pos"] = [p.x, p.y, p.z]
                item["vel"] = [v.x, v.y, v.z]
                
            self.history.append(item)
        except rospy.ServiceException:
            pass

def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= 1.0 
    return p

def execute_trajectory(robot, plan_data):
    if not plan_data: return False
    cmd_stream = [RobotCommand(p['position'], p['velocity'], p['acceleration']) for p in plan_data]
    robot.execute_stream(cmd_stream, ControlMode.TRAJECTORY)
    return True

def execute_toss(robot, gripper, sol, release_idx, recorder):
    Q, Qd, Qdd = sol["Q"], sol["Qd"], sol["Qdd"]
    N = Q.shape[0]
    robot._command_msg.mode = ControlMode.TRAJECTORY
    recorder.start()
    
    cmd_sent = False
    
    for i in range(N):
        if rospy.is_shutdown(): break
        
        if i == release_idx:
            gripper.open()
            cmd_sent = True
            
        cmd = RobotCommand(Q[i].tolist(), Qd[i].tolist(), Qdd[i].tolist())
        robot._command_msg.position = cmd.position
        robot._command_msg.velocity = cmd.velocity
        robot._command_msg.acceleration = cmd.acceleration
        robot._command_msg.header.stamp = rospy.Time.now()
        robot._pub_joint_cmd.publish(robot._command_msg)
        
        recorder.record_step(cmd_active=cmd_sent)
        robot._rate.sleep()
        
    for _ in range(100): 
        recorder.record_step(cmd_active=cmd_sent)
        rospy.sleep(0.01)
        
    return recorder.stop()

def run_timing_test():
    rospy.init_node('run_timing_test')
    rospy.loginfo("--- STARTING DETAILED RELEASE TIMING TEST ---")
    
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    recorder = TrajectoryRecorder("toss_cube", gripper)
    
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    # Test Matrix
    TEST_SPEEDS = [1.0, 1.5, 2.0]
    TEST_OFFSETS = [0, 2, 4, 6, 8, 10, 12] 
    
    full_log = []

    for speed in TEST_SPEEDS:
        for offset in TEST_OFFSETS:
            rospy.loginfo(f"\n>>> SPEED: {speed:.1f} m/s | OFFSET: {offset}")
            
            for trial in range(TRIALS_PER_CONFIG):
                # Reset
                manager.despawn("toss_cube")
                manager.spawn("cube", "toss_cube", Pose(Point(*PICK_POS_WORLD), Quaternion(0,0,0,1)))
                rospy.sleep(0.5)
                
                # Pick Logic (Condensed)
                traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
                execute_trajectory(robot, traj)
                gripper.open()
                
                pick_target = to_robot_frame(PICK_POS_WORLD)
                hover_target = list(pick_target); hover_target[2] += 0.05
                q_curr = robot.get_joint_positions()
                q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target, [0,1,0,0])
                if q_hover: execute_trajectory(robot, pick_planner.plan_joint(q_curr, q_hover, joint_speed=1.0))
                else: execute_trajectory(robot, pick_planner.plan_cartesian(q_curr, hover_target, [0,1,0,0], linear_speed=0.2))
                
                execute_trajectory(robot, pick_planner.plan_cartesian(robot.get_joint_positions(), pick_target, [0,1,0,0], linear_speed=0.1))
                gripper.close()
                rospy.sleep(0.5)
                execute_trajectory(robot, pick_planner.plan_cartesian(robot.get_joint_positions(), hover_target, [0,1,0,0], linear_speed=0.1))
                
                # Align & Plan
                j0_angle = np.deg2rad(ANGLE_DEG)
                q_ready = list(cfg.TOSS_READY_POS); q_ready[0] = j0_angle
                execute_trajectory(robot, pick_planner.plan_joint(robot.get_joint_positions(), q_ready, joint_speed=1.5))
                
                q_curr = np.array(robot.get_joint_positions())
                q0_3dof = np.array([q_curr[1], q_curr[3], q_curr[5]])
                tp = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
                sol_3d = tp.get_trajectory(speed)
                sol_7d = tp.map_to_7dof(sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"], q_curr[0], q_curr[2], q_curr[4], q_curr[6])
                
                # Execute
                sensor.start_listening()
                rel_idx_plan = max(0, sol_3d["index"] - offset)
                traj_data = execute_toss(robot, gripper, sol_7d, rel_idx_plan, recorder)
                
                # Analyze Gripper Delay
                cmd_idx = next((i for i, x in enumerate(traj_data) if x['cmd_active']), None)
                
                delay_time = 0.0
                grip_speed = 0.0
                flight_speed = 0.0
                
                if cmd_idx is not None:
                    t_cmd = traj_data[cmd_idx]['t']
                    w_start = traj_data[cmd_idx]['grip_width']
                    
                    # Find when width changed by > 1mm
                    motion_idx = next((i for i in range(cmd_idx, len(traj_data)) 
                                       if abs(traj_data[i]['grip_width'] - w_start) > 0.001), None)
                                       
                    if motion_idx:
                        t_motion = traj_data[motion_idx]['t']
                        delay_time = t_motion - t_cmd
                        
                        # Velocity Analysis
                        # Grip Speed: At moment of command
                        grip_speed = np.linalg.norm(traj_data[cmd_idx]['vel'])
                        
                        # Flight Speed: At moment of motion + buffer
                        flight_idx = min(motion_idx + 5, len(traj_data)-1)
                        flight_speed = np.linalg.norm(traj_data[flight_idx]['vel'])
                
                loss = grip_speed - flight_speed
                rospy.loginfo(f"Delay: {delay_time*1000:.1f}ms | Grip: {grip_speed:.2f} | Flight: {flight_speed:.2f} | Loss: {loss:.2f}")
                
                full_log.append({
                    "speed": speed, "offset": offset, 
                    "delay_ms": delay_time*1000, 
                    "loss": loss,
                    "grip_v": grip_speed,
                    "flight_v": flight_speed
                })
                
                # Reset
                traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
                execute_trajectory(robot, traj)
                rospy.sleep(0.5)
            
    with open(LOG_FILE, 'w') as f:
        json.dump(full_log, f, indent=2)
    rospy.loginfo(f"TIMING TEST COMPLETE. Saved to {LOG_FILE}")

if __name__ == "__main__":
    try:
        run_timing_test()
    except rospy.ROSInterruptException:
        pass