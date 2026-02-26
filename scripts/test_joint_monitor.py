#!/usr/bin/env python3
import os
import sys
import time
import cv2
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime



from sawyer_robot import SawyerRobot
from sawyer_common.geometry import JointAngles
from sawyer_motion_planner import CasadiKinematics, UnifiedPlanner
from tossingbot.utils.joint_monitor import JointMonitor
from tossingbot import config as cfg

def create_status_image(text):
    """Create a simple status image for the head display."""
    img = np.zeros((600, 1024, 3), np.uint8)
    img[:] = (40, 40, 40) # Dark gray background
    cv2.putText(img, text, (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)
    return img

def set_robot_color(robot, color):
    """Convenience function to set the head and right hand LEDs."""
    if not hasattr(robot, 'lights'): return
    r, g, b = False, False, False
    if color == 'red': r = True
    elif color == 'green': g = True
    elif color == 'blue': b = True
    elif color == 'yellow': r, g = True, True
    elif color == 'white': r, g, b = True, True, True
    try:
        robot.lights.set('head_red_light', r)
        robot.lights.set('head_green_light', g)
        robot.lights.set('head_blue_light', b)
        robot.lights.set('right_hand_red_light', r)
        robot.lights.set('right_hand_green_light', g)
        robot.lights.set('right_hand_blue_light', b)
    except: pass

def map_3to7(q3_traj, current_q7, active_indices):
    """Maps a 3-joint trajectory back to 7-joint space."""
    N = q3_traj.shape[0]
    q7_traj = np.tile(current_q7, (N, 1))
    for i, idx in enumerate(active_indices):
        q7_traj[:, idx] = q3_traj[:, i]
    return q7_traj

def main():
    parser = argparse.ArgumentParser(description="Test Joint Monitoring and Tossing")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode (no buttons)")
    parser.add_argument("--host", default="localhost", help="Robot host")
    args = parser.parse_args()

    # Paths
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    BASE_LINK = "base"
    TIP_LINK = "right_gripper_tip"
    MID_LINK = "right_hand"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_log_dir = os.path.join(project_root, "logs/joint_monitoring", timestamp)
    os.makedirs(run_log_dir, exist_ok=True)
    
    print(f"Connecting to robot at {args.host}...")
    
    with SawyerRobot(host=args.host) as robot:
        robot.enable()
        time.sleep(1.0)
        
        monitor = JointMonitor(robot, hz=100.0)
        
        # 1. PRE-COMPUTE TRAJECTORIES
        set_robot_color(robot, 'blue')
        robot.head.display_image(create_status_image("CALCULATING PATHS..."))
        
        # --- Kinematics & Planning Setup ---
        kin7 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
        planner7 = UnifiedPlanner(kin7)
        planner7.set_profile("slow") # Smooth move to ready
        
        curr_q = np.array(robot.arm.get_joints().to_list())
        ready_q = np.array(cfg.TOSS_READY_POS)
        
        print("\n--- Planning Phase 1: Coordinated move to Ready ---")
        p2p_sol = planner7.solve_p2p_joint(curr_q, ready_q)
        if not p2p_sol: 
            print("P2P solver failed!")
            return

        print("--- Planning Phase 2: 3-DOF Toss + Stop ---")
        active_indices = [1, 3, 5]
        joint_names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
        locked = {joint_names[i]: ready_q[i] for i in range(7) if i not in active_indices}
        
        kin3 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK, 
                               active_joints=["right_j1", "right_j3", "right_j5"], 
                               locked_joints=locked)
        planner3 = UnifiedPlanner(kin3)
        
        q3_start = ready_q[active_indices]
        # Target based on robot geometry
        target_pos = [0.85, 0.0, 0.5] 
        target_speed = 3.0
        release_angle = np.deg2rad(45)
        
        toss_sol = planner3.solve_toss(q3_start, target_pos, target_speed, release_angle, duration=0.7)
        if not toss_sol: 
            print("Toss solver failed!")
            return
            
        stop_sol = planner3.append_stop_trajectory(toss_sol['Q'][-1], toss_sol['Qd'][-1], stop_duration=0.8)
        
        Q3_full = np.vstack([toss_sol['Q'], stop_sol['Q']])
        Qd3_full = np.vstack([toss_sol['Qd'], stop_sol['Qd']])
        toss_stop_q7 = map_3to7(Q3_full, ready_q, active_indices)

        # 2. TRIGGER LOGIC
        if args.sim:
            print("\n--- SIM MODE: Bypassing buttons ---")
            time.sleep(1.0)
        else:
            print("\n--- Ready! Waiting for physical button input ---")
            robot.head.display_image(create_status_image("READY. PRESS SQUARE TO TOSS."))
            waiting_for_trigger = True
            try:
                while waiting_for_trigger:
                    nav = robot.navigator.get_state(side="all")
                    cuff = robot.cuff.get_state(side="right")
                    active_nav = {k: v for k, v in nav.items() if v != 'OFF' and 'wheel' not in k}
                    active_cuff = [k for k, v in cuff.items() if v]
                    
                    if "lower" in active_cuff:
                        if hasattr(robot, 'gripper'): robot.gripper.close() 
                        set_robot_color(robot, 'white'); time.sleep(0.5); set_robot_color(robot, 'blue')
                    elif "upper" in active_cuff:
                        if hasattr(robot, 'gripper'): robot.gripper.open()
                        set_robot_color(robot, 'white'); time.sleep(0.5); set_robot_color(robot, 'blue')
                    elif "right_button_square" in active_nav or "head_button_square" in active_nav:
                        waiting_for_trigger = False
                    time.sleep(0.1)
            except KeyboardInterrupt:
                return

        # 3. COUNTDOWN
        for i in range(3, 0, -1):
            set_robot_color(robot, 'red')
            robot.head.display_image(create_status_image(f"TOSSING IN {i}..."))
            time.sleep(0.5); set_robot_color(robot, 'off'); time.sleep(0.5)
            
        robot.head.display_image(create_status_image("EXECUTING TOSS!"))
        set_robot_color(robot, 'green')

        # 4. EXECUTE & MONITOR
        print("\n--- Execution Start ---")
        monitor.start_recording()
        monitor_start_time = time.time()
        
        # P2P Execution
        p2p_cmd_send_time = time.time() 
        robot.arm.stream_trajectory(p2p_sol['Q'], p2p_sol['Qd'], p2p_sol['Qdd'])
        time.sleep(0.2) 
        
        # Toss Execution
        toss_cmd_send_time = time.time() 
        release_idx = toss_sol['Q'].shape[0]
        # Command 100Hz trajectory
        robot.arm.stream_trajectory(toss_stop_q7, map_3to7(Qd3_full, np.zeros(7), active_indices), np.zeros_like(toss_stop_q7), release_index=release_idx)
        
        time.sleep(1.0) 
        monitor.stop_recording()
        print("--- Execution Complete ---")
        
        set_robot_color(robot, 'yellow')
        robot.head.display_image(create_status_image("DONE! SAVING LOGS..."))
        
        # 5. DATA ANALYSIS & PLOTTING
        data = monitor.get_data()
        if len(data['time']) == 0: return

        # Time offsets for plotting
        t_p2p_cmd = (p2p_cmd_send_time - monitor_start_time) + np.arange(len(p2p_sol['Q'])) * planner7.dt
        t_toss_cmd = (toss_cmd_send_time - monitor_start_time) + np.arange(len(toss_stop_q7)) * planner3.dt

        fig, axes = plt.subplots(3, 1, figsize=(10, 10))
        for idx, i in enumerate([1, 3, 5]):
            ax = axes[idx]
            ax.plot(data['time'], data['q'][:, i], 'b-', label='Actual')
            ax.plot(t_p2p_cmd, p2p_sol['Q'][:, i], 'r--', label='Cmd P2P')
            ax.plot(t_toss_cmd, toss_stop_q7[:, i], 'g--', label='Cmd Toss')
            ax.set_ylabel(f"J{i} Pos")
            ax.legend(fontsize='x-small')
            ax.grid(True, alpha=0.3)
        
        plt.xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(os.path.join(run_log_dir, "summary.png"))
        print(f"Plots saved to {run_log_dir}")
        
        set_robot_color(robot, 'off')
        robot.head.display_clear()

if __name__ == "__main__":
    main()
