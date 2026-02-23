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
from tossingbot.tossing.kinematics import RobotKinematics

# --- CONFIGURATION ---
SUITE_NAME = "target_accuracy_test"
LOG_DIR = os.path.join(cfg.PACKAGE_ROOT, "logs", "targets")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"{SUITE_NAME}_{int(time.time())}.json")

# Define Targets (X, Y) in World Frame
# Robot Base is at (0,0). Max reach/speed limits apply.
TARGETS = [
    (1.1, 0.0),
    (1.1, 0.15),
    (1.1, -0.15),
    (1.3, 0.0)
]

TRIALS_PER_TARGET = 3
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
SPAWN_BINS = False

# Constants
G = 9.806
TABLE_Z = 0.75
RELEASE_HEIGHT_W = 1.0
RELEASE_X_OFFSET = 0.825

# Inverse Ballistic Function
def calculate_required_speed(dist, drop_height):
    """
    Calculates required release speed for a 45-degree toss.
    v = sqrt( (g * d^2) / (d - dh) )
    where dh is drop height (negative if target is lower).
    """
    # d - dh -> distance + drop (since drop is negative -0.25, -dh is +0.25)
    # denominator = dist - drop_height 
    denominator = dist - drop_height
    if denominator <= 0:
        return None
    
    v_squared = (G * dist**2) / denominator
    return np.sqrt(v_squared)

def get_ballistic_prediction(release_pos, release_vel):
    """Forward prediction for error calculation."""
    x0, y0, z0 = release_pos
    vx, vy, vz = release_vel
    
    # Time to hit table: Solve z0 + vz*t - 0.5*g*t^2 = TABLE_Z
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
        
    for _ in range(50):
        recorder.record_step()
        rospy.sleep(0.01)
        
    return recorder.stop()

def run_targets():
    rospy.init_node('run_tossing_targets')
    rospy.loginfo("--- STARTING TARGET ACCURACY TEST ---")
    
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    recorder = TrajectoryRecorder("toss_cube")
    rk = RobotKinematics()
    
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    # Spawn bins for context? Or just tables?
    # Assuming standard environment is loaded.
    
    full_log = []

    for tx, ty in TARGETS:
        
        # 1. Calculate Required Parameters
        # A. Base Rotation (J0)
        j0 = TargetAlignment.get_base_rotation(tx, ty, cfg.CENTER_Y)
        angle_deg = np.rad2deg(j0)
        
        # B. Distance Calculation
        # Project target into "Unrotated" frame aligned with arm
        # Rotation Matrix R(-j0)
        # x_local = tx * cos(-j0) - ty * sin(-j0) = tx * cos(j0) + ty * sin(j0)
        x_local = tx * np.cos(j0) + ty * np.sin(j0)
        # y_local should be ~ CENTER_Y (0.1363)
        
        # Distance projectile needs to travel
        dist = x_local - RELEASE_X_OFFSET
        
        # C. Required Speed
        drop_h = TABLE_Z - RELEASE_HEIGHT_W # -0.25
        req_speed = calculate_required_speed(dist, drop_h)
        
        target_info = f"Target: ({tx:.2f}, {ty:.2f}) -> Dist: {dist:.3f}m, Speed: {req_speed:.3f} m/s, J0: {angle_deg:.1f} deg"
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo(target_info)
        
        if req_speed is None or req_speed > 2.0:
            rospy.logwarn("Target unreachable (Speed > 2.0 or invalid). Skipping.")
            continue

        for trial in range(TRIALS_PER_TARGET):
            rospy.loginfo(f"--- Trial {trial+1}/{TRIALS_PER_TARGET} ---")
            
            # Reset
            manager.despawn("toss_cube")
            manager.despawn("target_marker")
            manager.spawn("cube", "toss_cube", Pose(Point(*PICK_POS_WORLD), Quaternion(0,0,0,1)))
            
            # Spawn Target Marker at the GOAL
            manager.spawn("sphere", "target_marker", Pose(Point(tx, ty, 0.760), Quaternion(0,0,0,1)))
            rospy.sleep(0.5)
            
            # --- PICK SEQUENCE ---
            traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
            execute_trajectory(robot, traj)
            
            gripper.open()
            pick_target_r = to_robot_frame(PICK_POS_WORLD)
            hover_target_r = list(pick_target_r); hover_target_r[2] += 0.05
            
            q_curr = robot.get_joint_positions()
            q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target_r, [0,1,0,0])
            if q_hover:
                traj = pick_planner.plan_joint(q_curr, q_hover)
                execute_trajectory(robot, traj)
            else:
                rospy.logerr("Pick IK Failed")
                continue
                
            traj = pick_planner.plan_cartesian(robot.get_joint_positions(), pick_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
            execute_trajectory(robot, traj)
            
            gripper.close()
            rospy.sleep(0.5)
            
            traj = pick_planner.plan_cartesian(robot.get_joint_positions(), hover_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
            execute_trajectory(robot, traj)
            
            # --- ALIGNMENT ---
            q_ready = list(cfg.TOSS_READY_POS)
            q_ready[0] = j0
            traj = pick_planner.plan_joint(robot.get_joint_positions(), q_ready, joint_speed=1.5)
            execute_trajectory(robot, traj)
            
            # --- PLAN TOSS ---
            q_curr = np.array(robot.get_joint_positions())
            q0_3dof = np.array([q_curr[1], q_curr[3], q_curr[5]])
            tp = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
            sol_3d = tp.get_trajectory(req_speed)
            sol_7d = tp.map_to_7dof(sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"], q_curr[0], q_curr[2], q_curr[4], q_curr[6])
            
            # --- EXECUTE ---
            sensor.start_listening()
            traj_data = execute_toss(robot, gripper, sol_7d, sol_3d["index"] - 4, recorder)
            
            # --- RESULT ---
            land_pos, _ = sensor.get_landing_result(timeout=6.0)
            
            ballistic_error = -1
            target_error = -1
            
            if land_pos:
                real_pos = np.array([land_pos.x, land_pos.y])
                target_pos = np.array([tx, ty])
                target_error = np.linalg.norm(real_pos - target_pos) * 1000
                
                if traj_data:
                    # Use the specific release index
                    rel_idx = sol_3d["index"] - 4
                    if rel_idx < len(traj_data):
                        r_pos = traj_data[rel_idx]['pos']
                        r_vel = traj_data[rel_idx]['vel']
                        actual_speed = np.linalg.norm(r_vel)
                        
                        # Log Tracking Error
                        speed_err = actual_speed - req_speed
                        rospy.loginfo(f"Cmd Speed: {req_speed:.3f} m/s | Act Speed: {actual_speed:.3f} m/s | Diff: {speed_err:.3f} m/s")

                        pred = get_ballistic_prediction(r_pos, r_vel)
                        if pred is not None:
                            ballistic_error = np.linalg.norm(real_pos - pred) * 1000
                            # Move marker to PREDICTED spot to show consistency
                            manager.despawn("target_marker")
                            manager.spawn("sphere", "target_marker", Pose(Point(pred[0], pred[1], 0.760), Quaternion(0,0,0,1)))
                    else:
                        rospy.logwarn("Trajectory recording shorter than release index")
                
                rospy.loginfo(f"Target Error: {target_error:.2f} mm | Ballistic Consistency: {ballistic_error:.2f} mm")
            
            log_entry = {
                "target": {"x": tx, "y": ty},
                "params": {"speed": req_speed, "j0": j0},
                "release_idx": sol_3d["index"] - 4,
                "landing": {"x": land_pos.x if land_pos else None, "y": land_pos.y if land_pos else None},
                "error_target_mm": target_error,
                "error_ballistic_mm": ballistic_error
            }
            full_log.append(log_entry)
            
            # Return to neutral
            traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
            execute_trajectory(robot, traj)
            rospy.sleep(0.5)

    with open(LOG_FILE, 'w') as f:
        json.dump(full_log, f, indent=2)
    
    manager.despawn("target_marker")
    if SPAWN_BINS:
        manager.despawn("grid_bins")

if __name__ == "__main__":
    try:
        run_targets()
    except rospy.ROSInterruptException:
        pass
