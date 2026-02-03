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

# Offsets relative to end of trajectory
# 4 steps = 40ms, 20 steps = 200ms
RELEASE_OFFSETS = [4, 6, 8, 10, 12, 14, 16, 18, 20]
SPEED = 1.5
ANGLE_DEG = 0.0
TRIALS_PER_CONFIG = 3
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]

# Constants
G = 9.806
TABLE_Z = 0.75

def get_ballistic_prediction(release_pos, release_vel):
    x0, y0, z0 = release_pos
    vx, vy, vz = release_vel
    a_q = 0.5 * G
    b_q = -vz
    c_q = TABLE_Z - z0
    discriminant = b_q**2 - 4*a_q*c_q
    if discriminant < 0: return None
    tof = (vz + np.sqrt(vz**2 + 2*G*(z0 - TABLE_Z))) / G
    land_x = x0 + vx * tof
    land_y = y0 + vy * tof
    return np.array([land_x, land_y])

class TrajectoryRecorder:
    def __init__(self, object_name):
        self.object_name = object_name
        self.history = []
        self.recording = False
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def start(self):
        self.history = []
        self.recording = True
        
    def stop(self):
        self.recording = False
        return self.history

    def record_step(self):
        if not self.recording: return
        try:
            resp = self.get_state(self.object_name, "world")
            if resp.success:
                p = resp.pose.position
                v = resp.twist.linear
                self.history.append({
                    "t": rospy.get_time(),
                    "pos": [p.x, p.y, p.z],
                    "vel": [v.x, v.y, v.z]
                })
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
    for i in range(N):
        if rospy.is_shutdown(): break
        if i == release_idx:
            gripper.open()
            rospy.loginfo("RELEASE!")
        cmd = RobotCommand(Q[i].tolist(), Qd[i].tolist(), Qdd[i].tolist())
        robot._command_msg.position = cmd.position
        robot._command_msg.velocity = cmd.velocity
        robot._command_msg.acceleration = cmd.acceleration
        robot._command_msg.header.stamp = rospy.Time.now()
        robot._pub_joint_cmd.publish(robot._command_msg)
        recorder.record_step()
        robot._rate.sleep()
    for _ in range(100): # Record longer tail (1.0s)
        recorder.record_step()
        rospy.sleep(0.01)
    return recorder.stop()

def run_timing_test():
    rospy.init_node('run_timing_test')
    rospy.loginfo("--- STARTING RELEASE TIMING TEST ---")
    
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    recorder = TrajectoryRecorder("toss_cube")
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    full_log = []

    for offset in RELEASE_OFFSETS:
        rospy.loginfo(f"\n>>> TESTING OFFSET: {offset} steps ({offset*10}ms before end)")
        
        for trial in range(TRIALS_PER_CONFIG):
            manager.despawn("toss_cube")
            manager.spawn("cube", "toss_cube", Pose(Point(*PICK_POS_WORLD), Quaternion(0,0,0,1)))
            rospy.sleep(0.5)
            
            # Reset Pose
            traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
            execute_trajectory(robot, traj)
            
            # Pick
            gripper.open()
            pick_target_r = to_robot_frame(PICK_POS_WORLD)
            hover_target_r = [pick_target_r[0], pick_target_r[1], pick_target_r[2] + 0.05]
            q_curr = robot.get_joint_positions()
            q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target_r, [0,1,0,0])
            if q_hover: traj = pick_planner.plan_joint(q_curr, q_hover)
            else: traj = pick_planner.plan_cartesian(q_curr, hover_target_r, [0,1,0,0], linear_speed=0.2)
            execute_trajectory(robot, traj)
            traj = pick_planner.plan_cartesian(robot.get_joint_positions(), pick_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
            execute_trajectory(robot, traj)
            gripper.close()
            rospy.sleep(0.5)
            traj = pick_planner.plan_cartesian(robot.get_joint_positions(), hover_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
            execute_trajectory(robot, traj)
            
            # Align
            j0_angle = np.deg2rad(ANGLE_DEG)
            q_ready = list(cfg.TOSS_READY_POS)
            q_ready[0] = j0_angle
            traj = pick_planner.plan_joint(robot.get_joint_positions(), q_ready, joint_speed=1.5)
            execute_trajectory(robot, traj)
            
            # Toss
            q_curr = np.array(robot.get_joint_positions())
            q0_3dof = np.array([q_curr[1], q_curr[3], q_curr[5]])
            tp = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
            sol_3d = tp.get_trajectory(SPEED)
            sol_7d = tp.map_to_7dof(sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"], q_curr[0], q_curr[2], q_curr[4], q_curr[6])
            
            sensor.start_listening()
            # EXECUTE WITH VARIABLE OFFSET
            rel_idx = max(0, sol_3d["index"] - offset)
            traj_data = execute_toss(robot, gripper, sol_7d, rel_idx, recorder)
            
            land_pos, _ = sensor.get_landing_result(timeout=6.0)
            
            # Analyze
            grip_speed = 0
            flight_speed = 0
            loss = 0
            
            if traj_data and rel_idx < len(traj_data):
                grip_speed = np.linalg.norm(traj_data[rel_idx]['vel'])
                
                # Check 8 steps later (80ms)
                f_idx = min(rel_idx + 8, len(traj_data)-1)
                flight_speed = np.linalg.norm(traj_data[f_idx]['vel'])
                loss = grip_speed - flight_speed
                
                rospy.loginfo(f"Offset {offset}: Grip {grip_speed:.2f} -> Flight {flight_speed:.2f} (Loss: {loss:.2f})")
            
            log_entry = {
                "offset": offset,
                "grip_speed": grip_speed,
                "flight_speed": flight_speed,
                "loss": loss,
                "landing_x": land_pos.x if land_pos else None
            }
            full_log.append(log_entry)
            
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
