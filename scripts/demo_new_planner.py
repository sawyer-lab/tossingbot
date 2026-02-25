import os
import time
import numpy as np
from sawyer_robot import SawyerRobot
from sawyer_robot.geometry import JointAngles
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.planner import UnifiedPlanner
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
    
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to robot at {host}...")
    
    with SawyerRobot(host=host) as robot:
        print("Enabling robot...")
        robot.enable()
        robot.gripper.close()
        time.sleep(1.0)
        
        # --- PHASE 1: 7-DOF P2P TO READY POSITION ---
        print("\n--- Phase 1: Moving to Ready Position (7-DOF) ---")
        kin7 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
        planner7 = UnifiedPlanner(kin7)
        
        curr_q = np.array(robot.arm.get_joints().to_list())
        ready_q = np.array(cfg.TOSS_READY_POS)
        
        print(f"Solving P2P from current to Ready...")
        p2p_sol = planner7.solve_p2p_joint(curr_q, ready_q, duration=1.5)
        
        if p2p_sol:
            print("Executing P2P move...")
            robot.arm.stream_trajectory(p2p_sol['Q'], p2p_sol['Qd'], p2p_sol['Qdd'])
            time.sleep(1.5)
        else:
            print("P2P solver failed!")
            return

        # --- PHASE 2: 3-DOF TOSSING ---
        print("\n--- Phase 2: Executing 3-DOF Toss ---")
        curr_q7_list = robot.arm.get_joints().to_list()
        curr_q7 = np.array(curr_q7_list)
        joint_names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
        
        active_indices = [1, 3, 5] # J1, J3, J5
        tossing_joints = ["right_j1", "right_j3", "right_j5"]
        
        # Lock inactive joints
        locked = {joint_names[i]: curr_q7_list[i] for i in range(7) if i not in active_indices}
        
        kin3 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK, 
                                active_joints=tossing_joints,
                                locked_joints=locked)
        planner3 = UnifiedPlanner(kin3)
        curr_q3 = curr_q7[active_indices]
        
        # FIXED POSITIONS (from legacy logic)
        # Based on legacy ROBOT_PARAMS: base_offset=[0.081, 0.0, 0.317], targetPosition=[0.70, 0.0, -angle]
        # x_target = 0.081 + 0.70 = 0.781
        # z_target = 0.317 + 0.0 = 0.317
        
        # Get current Y to stay in plane
        current_y = float(kin3.fk_pos(curr_q3)[1])
        target_pos = [0.781, current_y, 0.317]
        
        target_speed = 1.8 # m/s
        release_angle = np.deg2rad(45)
        
        print(f"Solving Toss to {target_pos} @ {target_speed} m/s...")
        toss_sol = planner3.solve_toss(curr_q3, target_pos, target_speed, release_angle, duration=0.7)
        
        if toss_sol:
            Q7 = map_3to7(toss_sol['Q'], curr_q7, active_indices)
            Qd7 = map_3to7(toss_sol['Qd'], np.zeros(7), active_indices)
            Qdd7 = map_3to7(toss_sol['Qdd'], np.zeros(7), active_indices)
            
            release_idx = toss_sol['Q'].shape[0]  + 5
            stop_sol = planner3.append_stop_trajectory(toss_sol['Q'][-1], toss_sol['Qd'][-1], stop_duration=0.8)
            
            if stop_sol:
                Q7_stop = map_3to7(stop_sol['Q'], curr_q7, active_indices)
                Qd7_stop = map_3to7(stop_sol['Qd'], np.zeros(7), active_indices)
                Qdd7_stop = map_3to7(stop_sol['Qdd'], np.zeros(7), active_indices)
                Q_full = np.vstack([Q7, Q7_stop]); Qd_full = np.vstack([Qd7, Qd7_stop]); Qdd_full = np.vstack([Qdd7, Qdd7_stop])
            else:
                Q_full, Qd_full, Qdd_full = Q7, Qd7, Qdd7

            print(f"Executing Toss with release at index {release_idx}...")
            robot.arm.stream_trajectory(Q_full, Qd_full, Qdd_full, release_index=release_idx)
            print("Toss complete.")
        else:
            print("Toss solver failed! Check reachability for target_pos.")

        # # Return to Ready
        # print("\n--- Returning to Ready position ---")
        # curr_q = np.array(robot.arm.get_joints().to_list())
        # reset_sol = planner7.solve_p2p_joint(curr_q, ready_q, duration=1.0)
        # if reset_sol:
        #     robot.arm.stream_trajectory(reset_sol['Q'], reset_sol['Qd'], reset_sol['Qdd'])

if __name__ == "__main__":
    main()
