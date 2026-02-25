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
SUITE_NAME = "speed_angle_dispersion_v1"
LOG_DIR = os.path.join(cfg.PACKAGE_ROOT, "logs", "test_suite")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"{SUITE_NAME}_{int(time.time())}.json")

# Offsets relative to CENTER_Y (0.1363)
Y_OFFSETS = [0.0, 0.15, -0.15] 
SPEEDS = np.linspace(1.0, 2.0, 5).tolist()  # [1.0, 1.25, 1.5, 1.75, 2.0]
ANGLES_DEG = np.linspace(-8, 8, 5).tolist() # [-8, -4, 0, 4, 8]
TRIALS_PER_CONFIG = 1

# Pick Position (Using shifted Y coordinate)
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
BIN_CENTER_X = 1.10
BIN_CENTER_Z = 0.760

# Test Flags
SPAWN_BINS = False

# Constants for Ballistic Calc
G = 9.806
TABLE_Z = 0.75

def get_ballistic_prediction(release_pos, release_vel):
    """Calculates landing position based on projectile motion."""
    x0, y0, z0 = release_pos
    vx, vy, vz = release_vel
    
    # Time to hit table: Solve z0 + vz*t - 0.5*g*t^2 = TABLE_Z
    # 0.5*G*t^2 - vz*t + (TABLE_Z - z0) = 0
    a_q = 0.5 * G
    b_q = -vz
    c_q = TABLE_Z - z0
    
    discriminant = b_q**2 - 4*a_q*c_q
    if discriminant < 0: return None
    
    tof = (vz + np.sqrt(vz**2 + 2*G*(z0 - TABLE_Z))) / G
    
    # Calculate landing position
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
    p[2] -= 1.0 # Robot Z Offset
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
        
        # Release Logic
        if i == release_idx:
            gripper.open()
            rospy.loginfo("RELEASE!")
            
        cmd = RobotCommand(Q[i].tolist(), Qd[i].tolist(), Qdd[i].tolist())
        robot._command_msg.position = cmd.position
        robot._command_msg.velocity = cmd.velocity
        robot._command_msg.acceleration = cmd.acceleration
        robot._command_msg.header.stamp = rospy.Time.now()
        robot._pub_joint_cmd.publish(robot._command_msg)
        
        # Record trajectory at ~100Hz (sim rate)
        recorder.record_step()
        robot._rate.sleep()
        
    # Continue recording for a bit after throw
    for _ in range(50): # 0.5s
        recorder.record_step()
        rospy.sleep(0.01)
        
    return recorder.stop()

def run_suite():
    rospy.init_node('run_tossing_suite')
    rospy.loginfo("--- STARTING SPEED/ANGLE DISPERSION SUITE ---")
    
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    recorder = TrajectoryRecorder("toss_cube")
    
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    if SPAWN_BINS:
        rospy.loginfo("Spawning Grid Bins...")
        manager.spawn("grid_bins", "grid_bins", 
            Pose(position=Point(1.10, cfg.CENTER_Y, 0.760), orientation=Quaternion(0,0,0,1))
        )
        rospy.sleep(1.0)
    else:
        rospy.loginfo("Skipping Grid Bins Spawn (Physics Test Mode)")

    full_log = []

    # --- MAIN LOOP ---
    for speed in SPEEDS:
        for angle_deg in ANGLES_DEG:
            config_name = f"Speed_{speed:.2f}_Angle_{angle_deg:.1f}"
            
            for trial in range(TRIALS_PER_CONFIG):
                rospy.loginfo(f"\n>>> CONFIG: {config_name} | TRIAL {trial+1}/{TRIALS_PER_CONFIG}")
                
                # 1. Reset
                manager.despawn("toss_cube")
                manager.spawn("cube", "toss_cube", Pose(Point(*PICK_POS_WORLD), Quaternion(0,0,0,1)))
                
                j0_angle = np.deg2rad(angle_deg)
                rospy.sleep(0.5)
                
                # Move to Neutral
                traj = pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
                execute_trajectory(robot, traj)
                
                # 2. Pick
                gripper.open()
                pick_target_r = to_robot_frame(PICK_POS_WORLD)
                hover_target_r = [pick_target_r[0], pick_target_r[1], pick_target_r[2] + 0.05]
                
                q_curr = robot.get_joint_positions()
                
                # IK
                q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target_r, [0,1,0,0])
                
                if q_hover:
                    traj = pick_planner.plan_joint(q_curr, q_hover)
                else:
                    rospy.logwarn("Hover IK failed, falling back to Cartesian...")
                    traj = pick_planner.plan_cartesian(q_curr, hover_target_r, [0,1,0,0], linear_speed=0.2)
                
                if not execute_trajectory(robot, traj):
                    rospy.logerr("Hover Plan Failed! Aborting trial.")
                    continue
                
                traj = pick_planner.plan_cartesian(robot.get_joint_positions(), pick_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
                if not execute_trajectory(robot, traj):
                    rospy.logerr("Approach Plan Failed! Aborting trial.")
                    continue
                
                gripper.close()
                rospy.sleep(0.5)
                
                traj = pick_planner.plan_cartesian(robot.get_joint_positions(), hover_target_r, [0,1,0,0], duration=None, linear_speed=0.1)
                if not execute_trajectory(robot, traj):
                    rospy.logerr("Lift Plan Failed! Aborting trial.")
                    continue
                
                # 3. Align Base
                j0_angle = np.deg2rad(angle_deg)
                q_ready = list(cfg.TOSS_READY_POS)
                q_ready[0] = j0_angle
                traj = pick_planner.plan_joint(robot.get_joint_positions(), q_ready, joint_speed=1.5)
                execute_trajectory(robot, traj)
                
                # 4. Plan Toss
                q_curr = np.array(robot.get_joint_positions())
                q0_3dof = np.array([q_curr[1], q_curr[3], q_curr[5]])
                tp = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
                sol_3d = tp.get_trajectory(speed)
                sol_7d = tp.map_to_7dof(sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"], q_curr[0], q_curr[2], q_curr[4], q_curr[6])
                
                # 5. Execute & Record
                rel_offset = 12
                sensor.start_listening()
                traj_data = execute_toss(robot, gripper, sol_7d, sol_3d["index"] - rel_offset, recorder)
                
                # 6. Result
                land_pos, obj_name = sensor.get_landing_result(timeout=6.0)
                
                # Correct release index based on execution
                rel_idx = sol_3d["index"] - rel_offset
                
                ballistic_error_mm = -1
                if land_pos and traj_data:
                    if rel_idx < len(traj_data):
                        # 1. Gripper Speed (At exact moment of open command)
                        r_vel_grip = traj_data[rel_idx]['vel']
                        grip_speed = np.linalg.norm(r_vel_grip)

                        # 2. Flight Speed (80ms later, once free)
                        flight_idx = min(rel_idx + 8, len(traj_data) - 1)
                        r_pos = traj_data[flight_idx]['pos']
                        r_vel = traj_data[flight_idx]['vel']
                        flight_speed = np.linalg.norm(r_vel)
                        
                        # Calculate Flight Geometry
                        v_horiz = np.linalg.norm(r_vel[:2])
                        flight_angle_deg = np.rad2deg(np.arctan2(r_vel[2], v_horiz))
                        flight_z = r_pos[2]
                        
                        loss = grip_speed - flight_speed
                        
                        pred_pos = get_ballistic_prediction(r_pos, r_vel)
                        
                        # Commanded Prediction
                        vx_p = speed * np.cos(np.deg2rad(45))
                        vz_p = speed * np.sin(np.deg2rad(45))
                        px0 = 0.825 * np.cos(j0_angle) - cfg.CENTER_Y * np.sin(j0_angle)
                        py0 = 0.825 * np.sin(j0_angle) + cfg.CENTER_Y * np.cos(j0_angle)
                        vx0 = vx_p * np.cos(j0_angle)
                        vy0 = vx_p * np.sin(j0_angle)
                        
                        # Using flight_z instead of 1.0 for fairer comparison
                        pred_cmd = get_ballistic_prediction([px0, py0, flight_z], [vx0, vy0, vz_p])
                        
                        if pred_pos is not None:
                            real_pos = np.array([land_pos.x, land_pos.y])
                            ballistic_error_mm = np.linalg.norm(real_pos - pred_pos) * 1000
                            
                            cmd_error_mm = 0
                            if pred_cmd is not None:
                                cmd_error_mm = np.linalg.norm(real_pos - pred_cmd) * 1000
                            
                            speed_diff = flight_speed - speed
                            angle_diff = flight_angle_deg - 45.0
                            
                            rospy.loginfo(f"Grip Speed: {grip_speed:.2f} m/s -> Flight Speed: {flight_speed:.2f} m/s (Loss: {loss:.2f} m/s)")
                            rospy.loginfo(f"Flight Angle: {flight_angle_deg:.1f} deg (Cmd: 45.0, Diff: {angle_diff:.1f})")
                            rospy.loginfo(f"Flight Z: {flight_z:.3f} m")
                            rospy.loginfo(f"SPEED DIFF: {speed_diff:.3f} m/s")
                            rospy.loginfo(f"ACT ERROR (Physics): {ballistic_error_mm:.2f} mm")
                            rospy.loginfo(f"CMD ERROR (Control): {cmd_error_mm:.2f} mm")
                        else:
                             rospy.logwarn("Ballistic prediction failed (discriminant < 0)")
                    else:
                        rospy.logwarn("Trajectory recording shorter than release index")
                
                result = {
                    "config": {"speed": speed, "angle_deg": angle_deg},
                    "j0_angle": j0_angle,
                    "release_idx": rel_idx,
                    "landing": {"x": land_pos.x if land_pos else None, "y": land_pos.y if land_pos else None, "success": bool(land_pos)},
                    "trajectory": traj_data
                }
                full_log.append(result)
                
                # Save incrementally
                with open(LOG_FILE, 'w') as f:
                   json.dump(full_log, f, indent=2)

    rospy.loginfo(f"SUITE COMPLETE. Saved to {LOG_FILE}")
    if SPAWN_BINS:
        manager.despawn("grid_bins")

if __name__ == "__main__":
    try:
        run_suite()
    except rospy.ROSInterruptException:
        pass
