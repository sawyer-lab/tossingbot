import os
import numpy as np
from sawyer_motion_planner import CasadiKinematics, UnifiedPlanner
from tossingbot import config as cfg

def main():
    # 1. Setup URDF path
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    
    BASE_LINK = "right_arm_base_link"
    TIP_LINK = "right_gripper_tip"
    
    print(f"Loading URDF: {urdf_path}")
    
    # 2. Test 7-DOF Mode (Point-to-Point)
    kin7 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
    planner7 = UnifiedPlanner(kin7)
    
    print(f"7-DOF Mode initialized. DOF: {kin7.n_dof}")
    
    q_start = np.array(cfg.NEUTRAL_JOINT_POS)
    q_end = np.array(cfg.TOSS_READY_POS)
    
    print("Solving 7-DOF P2P...")
    traj7 = planner7.solve_p2p_joint(q_start, q_end, duration=1.0)
    if traj7:
        print(f"Success! Trajectory shape: {traj7['Q'].shape}")
    else:
        print("Failed to solve 7-DOF P2P.")

    # 3. Test 3-DOF Tossing Mode
    # Use J1, J3, J5
    tossing_joints = ["right_j1", "right_j3", "right_j5"]
    kin3 = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK, active_joints=tossing_joints)
    planner3 = UnifiedPlanner(kin3)
    
    print(f"3-DOF Tossing Mode initialized. DOF: {kin3.n_dof}")
    
    # Starting position for 3 joints
    q3_start = np.array([-1.0, 1.5, -0.5]) 
    
    # Get initial position
    initial_pos = kin3.fk_pos(q3_start)
    print(f"Initial Pos: {initial_pos}")

    # Set target to EXACTLY the same Y as the initial pos
    # Initial Pos: [0.991454, 0.1603, 0.461816]
    target_pos = [0.8, 0.1603, 0.4] 
    target_speed = 1.0 
    release_angle = np.deg2rad(30)
    
    print(f"Solving 3-DOF Toss to {target_pos} @ {target_speed} m/s...")
    traj3 = planner3.solve_toss(q3_start, target_pos, target_speed, release_angle)
    if traj3:
        print(f"Success! Tossing trajectory shape: {traj3['Q'].shape}")
        last_q = traj3['Q'][-1]
        actual_pos = kin3.fk_pos(last_q)
        print(f"Actual End Pos: {actual_pos}")
    else:
        print("Failed to solve 3-DOF Toss. The solver still cannot find a valid solution for these 3 joints.")

if __name__ == "__main__":
    main()
