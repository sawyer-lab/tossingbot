import os
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from sawyer_robot import SawyerRobot
from sawyer_robot.geometry import JointAngles
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.planner import UnifiedPlanner
from tossingbot.utils.joint_monitor import JointMonitor
from tossingbot import config as cfg

def map_3to7(q3_traj, current_q7, active_indices):
    """Maps a 3-joint trajectory back to 7-joint space."""
    N = q3_traj.shape[0]
    q7_traj = np.tile(current_q7, (N, 1))
    for i, idx in enumerate(active_indices):
        q7_traj[:, idx] = q3_traj[:, i]
    return q7_traj

def main():
    project_root = "/home/fausto/Projects/sawyer/tossingbot"
    urdf_path = os.path.join(project_root, "assets/urdf/sawyer_tabletop_pneumatic.urdf")
    BASE_LINK = "right_arm_base_link"
    TIP_LINK = "right_gripper_tip"
    MID_LINK = "right_hand"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_log_dir = os.path.join(project_root, "logs/joint_monitoring", timestamp)
    os.makedirs(run_log_dir, exist_ok=True)
    
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to robot at {host}...")
    
    with SawyerRobot(host=host) as robot:
        robot.enable()
        time.sleep(1.0)
        
        
        monitor = JointMonitor(robot, hz=100.0)
        
        # Initialize kinematics for both the end-effector and the mid-link
        kin7 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
        kin7_mid = CasadiKinematics(urdf_path, BASE_LINK, MID_LINK)
        planner7 = UnifiedPlanner(kin7)
        
        # 1. PLAN P2P TO READY
        print("\n--- Planning Phase 1: P2P to Ready ---")
        curr_q = np.array(robot.arm.get_joints().to_list())
        ready_q = np.array(cfg.TOSS_READY_POS)
        p2p_sol = planner7.solve_p2p_joint(curr_q, ready_q, duration=10.0)
        if not p2p_sol: print("P2P solver failed!"); return

        # 2. PLAN TOSS + STOP
        print("--- Planning Phase 2: 3-DOF Toss + Stop ---")
        active_indices = [1, 3, 5]
        joint_names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
        locked = {joint_names[i]: ready_q[i] for i in range(7) if i not in active_indices}
        
        kin3 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK, active_joints=["right_j1", "right_j3", "right_j5"], locked_joints=locked)
        planner3 = UnifiedPlanner(kin3)
        
        q3_start = ready_q[active_indices]
        initial_pos = kin3.fk_pos(q3_start)
        target_y = float(initial_pos[1])
        target_pos = [0.825, target_y, 0.0]
        target_speed = 2.0
        release_angle = np.deg2rad(45)
        
        toss_sol = planner3.solve_toss(q3_start, target_pos, target_speed, release_angle, duration=0.7)
        if not toss_sol: print("Toss solver failed!"); return
            
        stop_sol = planner3.append_stop_trajectory(toss_sol['Q'][-1], toss_sol['Qd'][-1], stop_duration=0.8)
        
        Q3_full = np.vstack([toss_sol['Q'], stop_sol['Q']])
        Qd3_full = np.vstack([toss_sol['Qd'], stop_sol['Qd']])
        
        toss_stop_q7 = map_3to7(Q3_full, ready_q, active_indices)

        # 3. EXECUTE & MONITOR
        print("\n--- Execution Start ---")
        monitor.start_recording()
        monitor_start_time = time.time()
        
        # P2P Execution
        time.sleep(0.1) 
        p2p_cmd_send_time = time.time() 
        robot.arm.stream_trajectory(p2p_sol['Q'], p2p_sol['Qd'], p2p_sol['Qdd'])
        time.sleep(0.5) 
        
        # Toss Execution
        toss_cmd_send_time = time.time() 
        release_idx = toss_sol['Q'].shape[0] + 5
        robot.arm.stream_trajectory(toss_stop_q7, map_3to7(Qd3_full, np.zeros(7), active_indices), np.zeros((len(Q3_full), 7)), release_index=release_idx)
        
        time.sleep(1.0) 
        monitor.stop_recording()
        print("--- Execution Complete ---")
        
        # Calculate time vectors
        p2p_dt = planner7.dt
        t_p2p_cmd = (p2p_cmd_send_time - monitor_start_time) + np.arange(p2p_sol['Q'].shape[0]) * p2p_dt
        p2p_end_cmd_time = t_p2p_cmd[-1]
        
        toss_dt = planner3.dt
        t_toss_cmd = (toss_cmd_send_time - monitor_start_time) + np.arange(toss_stop_q7.shape[0]) * toss_dt
        cmd_release_time_absolute = t_toss_cmd[release_idx]

        # 4. DATA ANALYSIS & PLOTTING
        data = monitor.get_data()
        
        if len(data['time']) == 0:
            print("Error: No data was collected by the monitor. Plots will be empty.")
            return

        print(f"Collected {len(data['time'])} samples.")
        
        active_plot_joints = [1, 3, 5]
        
        # Figure for Joint Positions
        fig_pos, axes_pos = plt.subplots(len(active_plot_joints), 1, figsize=(12, 12), sharex=True)
        for idx, i in enumerate(active_plot_joints):
            ax = axes_pos[idx]
            ax.plot(data['time'], data['q'][:, i], 'b-', alpha=0.7, label=f'Actual J{i}')
            ax.plot(t_p2p_cmd, p2p_sol['Q'][:, i], 'r--', linewidth=1.0, label='Cmd P2P')
            ax.plot(t_toss_cmd, toss_stop_q7[:, i], 'g--', linewidth=1.0, label='Cmd Toss+Stop')
            ax.axvline(x=p2p_end_cmd_time, color='gray', linestyle='--', label='P2P Cmd End')
            ax.axvline(x=cmd_release_time_absolute, color='m', linestyle=':', label='Cmd Release')
            ax.set_ylabel(f"J{i} Pos (rad)")
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True, alpha=0.2)
        axes_pos[-1].set_xlabel("Time (s)")
        fig_pos.suptitle(f"Joint Positions - Run {timestamp}\n(P2P -> Toss -> Brake)", y=0.99)
        fig_pos.tight_layout(rect=[0, 0.03, 1, 0.96])
        fig_pos.savefig(os.path.join(run_log_dir, "joint_positions.png"))
        plt.close(fig_pos)

        # Figure for Joint Velocities
        fig_vel, axes_vel = plt.subplots(len(active_plot_joints), 1, figsize=(12, 12), sharex=True)
        for idx, i in enumerate(active_plot_joints):
            ax = axes_vel[idx]
            ax.plot(data['time'], data['qd'][:, i], 'b-', alpha=0.7, label=f'Actual J{i}d')
            ax.plot(t_p2p_cmd, p2p_sol['Qd'][:, i], 'r--', linewidth=1.0, label='Cmd P2P Vel')
            ax.plot(t_toss_cmd, map_3to7(Qd3_full, np.zeros(7), active_indices)[:, i], 'g--', linewidth=1.0, label='Cmd Toss+Stop Vel')
            ax.axvline(x=p2p_end_cmd_time, color='gray', linestyle='--', label='P2P Cmd End')
            ax.axvline(x=cmd_release_time_absolute, color='m', linestyle=':', label='Cmd Release')
            ax.set_ylabel(f"J{i} Vel (rad/s)")
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True, alpha=0.2)
        axes_vel[-1].set_xlabel("Time (s)")
        fig_vel.suptitle(f"Joint Velocities - Run {timestamp}\n(P2P -> Toss -> Brake)", y=0.99)
        fig_vel.tight_layout(rect=[0, 0.03, 1, 0.96])
        fig_vel.savefig(os.path.join(run_log_dir, "joint_velocities.png"))
        plt.close(fig_vel)
        
        # Expected Tip Endpoint Positions
        p2p_endp_pos = np.zeros((p2p_sol['Q'].shape[0], 3))
        for i in range(p2p_sol['Q'].shape[0]):
            p2p_endp_pos[i] = kin7.fk_pos(p2p_sol['Q'][i]).full().flatten()
            
        toss_stop_endp_pos = np.zeros((toss_stop_q7.shape[0], 3))
        for i in range(toss_stop_q7.shape[0]):
            toss_stop_endp_pos[i] = kin7.fk_pos(toss_stop_q7[i]).full().flatten()
            
        # Figure for Tip Endpoint Positions
        fig_endPOS, axes_endPOS = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        labels = ['X', 'Y', 'Z']
        colors = ['r', 'g', 'b']
        for i in range(3):
            ax = axes_endPOS[i]
            if 'endpoint_pos' in data and data['endpoint_pos'].size > 0:
                ax.plot(data['time'], data['endpoint_pos'][:, i], f'{colors[i]}-', alpha=0.7, label=f'Actual {labels[i]} Pos')
            ax.plot(t_p2p_cmd, p2p_endp_pos[:, i], 'r--', linewidth=1.0, label='Cmd P2P Pos')
            ax.plot(t_toss_cmd, toss_stop_endp_pos[:, i], 'g--', linewidth=1.0, label='Cmd Toss+Stop Pos')
            ax.axvline(x=p2p_end_cmd_time, color='gray', linestyle='--', label='P2P Cmd End')
            ax.axvline(x=cmd_release_time_absolute, color='m', linestyle=':', label='Cmd Release')
            ax.set_ylabel(f"{labels[i]} Pos (m)")
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True, alpha=0.2)
        axes_endPOS[-1].set_xlabel("Time (s)")
        fig_endPOS.suptitle(f"End-Effector Cartesian Positions - Run {timestamp}", y=0.99)
        fig_endPOS.tight_layout(rect=[0, 0.03, 1, 0.96])
        fig_endPOS.savefig(os.path.join(run_log_dir, "endpoint_positions.png"))
        plt.close(fig_endPOS)
        
        # Compute expected tip endpoint velocities
        p2p_endp_vel = np.zeros((p2p_sol['Q'].shape[0], 3))
        for i in range(p2p_sol['Q'].shape[0]):
            J = kin7.jacobian(p2p_sol['Q'][i])
            p2p_endp_vel[i] = (J @ p2p_sol['Qd'][i]).full().flatten()
            
        # Mapped Qd full used for both tip and mid link
        Qd7_full = map_3to7(Qd3_full, np.zeros(7), active_indices)
            
        toss_stop_endp_vel = np.zeros((toss_stop_q7.shape[0], 3))
        toss_stop_mid_vel = np.zeros((toss_stop_q7.shape[0], 3)) # NEW: Mid-link cmd velocities
        
        for i in range(toss_stop_q7.shape[0]):
            # Tip velocity
            J_tip = kin7.jacobian(toss_stop_q7[i])
            toss_stop_endp_vel[i] = (J_tip @ Qd7_full[i]).full().flatten()
            
            # Mid-link velocity
            J_mid = kin7_mid.jacobian(toss_stop_q7[i])
            toss_stop_mid_vel[i] = (J_mid @ Qd7_full[i]).full().flatten()
            
        # Compute "computed actual" endpoint and mid-link velocities
        computed_endp_vel = np.zeros_like(data['endpoint_vel'])
        computed_mid_vel = np.zeros((data['q'].shape[0], 3)) # NEW: Mid-link actual velocities
        
        if data['q'].shape[0] > 0 and data['qd'].shape[0] > 0:
            for i in range(data['q'].shape[0]):
                q_actual = data['q'][i]
                qd_actual = data['qd'][i]
                
                # Actual tip
                J_tip = kin7.jacobian(q_actual)
                computed_endp_vel[i] = (J_tip @ qd_actual).full().flatten()
                
                # Actual mid-link
                J_mid = kin7_mid.jacobian(q_actual)
                computed_mid_vel[i] = (J_mid @ qd_actual).full().flatten()
            
        vel_plots = [
            ('X', 0, 'r', 'X'),
            ('Y', 1, 'g', 'Y'),
            ('Z', 2, 'b', 'Z'),
            ('Mag XZ', -1, 'm', 'Mag')
        ]
        
        # Figure for Tip Endpoint Velocities
        fig_endp, axes_endp = plt.subplots(len(vel_plots), 1, figsize=(12, 12), sharex=True)
        for p_idx, (title, i, color, label) in enumerate(vel_plots):
            ax = axes_endp[p_idx]
            if i == -1: # Mag XZ
                if 'endpoint_vel' in data and data['endpoint_vel'].size > 0:
                    actual_mag = np.sqrt(data['endpoint_vel'][:, 0]**2 + data['endpoint_vel'][:, 2]**2)
                    ax.plot(data['time'], actual_mag, f'{color}-', alpha=0.7, label=f'Reported Actual {label} Vel')
                
                if computed_endp_vel.shape[0] > 0:
                    comp_mag = np.sqrt(computed_endp_vel[:, 0]**2 + computed_endp_vel[:, 2]**2)
                    ax.plot(data['time'], comp_mag, 'y-', linewidth=1.5, alpha=0.9, label=f'Computed Actual {label} Vel')
                    
                p2p_mag = np.sqrt(p2p_endp_vel[:, 0]**2 + p2p_endp_vel[:, 2]**2)
                toss_mag = np.sqrt(toss_stop_endp_vel[:, 0]**2 + toss_stop_endp_vel[:, 2]**2)
                
                ax.plot(t_p2p_cmd, p2p_mag, 'r--', linewidth=1.0, label='Cmd P2P Mag')
                ax.plot(t_toss_cmd, toss_mag, 'g--', linewidth=1.0, label='Cmd Toss+Stop Mag')
            else:
                if 'endpoint_vel' in data and data['endpoint_vel'].size > 0:
                    ax.plot(data['time'], data['endpoint_vel'][:, i], f'{color}-', alpha=0.7, label=f'Reported Actual {label} Vel')
                    
                if computed_endp_vel.shape[0] > 0:
                    ax.plot(data['time'], computed_endp_vel[:, i], 'y-', linewidth=1.5, alpha=0.9, label=f'Computed Actual {label} Vel')
                    
                ax.plot(t_p2p_cmd, p2p_endp_vel[:, i], 'r--', linewidth=1.0, label=f'Cmd P2P {label}')
                ax.plot(t_toss_cmd, toss_stop_endp_vel[:, i], 'g--', linewidth=1.0, label=f'Cmd Toss+Stop {label}')
                
            ax.axvline(x=p2p_end_cmd_time, color='gray', linestyle='--', label='P2P Cmd End')
            ax.axvline(x=cmd_release_time_absolute, color='k', linestyle=':', label='Cmd Release')
            ax.set_ylabel(f"{title} Vel (m/s)")
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True, alpha=0.2)
            
        axes_endp[-1].set_xlabel("Time (s)")
        fig_endp.suptitle(f"End-Effector Cartesian Velocities - Run {timestamp}", y=0.99)
        fig_endp.tight_layout(rect=[0, 0.03, 1, 0.96])
        fig_endp.savefig(os.path.join(run_log_dir, "endpoint_velocities.png"))
        plt.close(fig_endp)
        
        fig_mid, axes_mid = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        labels = ['X', 'Y', 'Z', 'Speed']
        colors = ['r', 'g', 'b', 'm']
        
        for i in range(4):
            ax = axes_mid[i]
            
            if i < 3:
                # Plot X, Y, Z components
                if computed_mid_vel.shape[0] > 0:
                    ax.plot(data['time'], computed_mid_vel[:, i], 'y-', linewidth=1.5, alpha=0.9, label=f'Actual {labels[i]} Vel')
                    
                ax.plot(t_toss_cmd, toss_stop_mid_vel[:, i], f'{colors[i]}--', linewidth=1.5, label=f'Cmd Toss+Stop {labels[i]}')
            else:
                # Plot Total Speed (Magnitude)
                if computed_mid_vel.shape[0] > 0:
                    actual_speed = np.sqrt(computed_mid_vel[:, 0]**2 + computed_mid_vel[:, 1]**2 + computed_mid_vel[:, 2]**2)
                    ax.plot(data['time'], actual_speed, 'y-', linewidth=1.5, alpha=0.9, label='Actual Speed')
                    
                cmd_speed = np.sqrt(toss_stop_mid_vel[:, 0]**2 + toss_stop_mid_vel[:, 1]**2 + toss_stop_mid_vel[:, 2]**2)
                ax.plot(t_toss_cmd, cmd_speed, f'{colors[i]}--', linewidth=1.5, label='Cmd Toss+Stop Speed')
            
            ax.axvline(x=cmd_release_time_absolute, color='k', linestyle=':', label='Cmd Release')
            ax.set_ylabel(f"{labels[i]} (m/s)")
            ax.legend(loc='upper right', fontsize='x-small')
            ax.grid(True, alpha=0.2)
            
        axes_mid[-1].set_xlabel("Time (s)")
        fig_mid.suptitle(f"{MID_LINK} Cartesian Velocities & Speed (Toss Phase) - Run {timestamp}", y=0.99)
        fig_mid.tight_layout(rect=[0, 0.03, 1, 0.96])
        fig_mid.savefig(os.path.join(run_log_dir, "mid_link_toss_velocities.png"))
        plt.close(fig_mid)
        
        print(f"All plots saved to {run_log_dir}/")

if __name__ == "__main__":
    main()