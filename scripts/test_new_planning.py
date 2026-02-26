import os
import numpy as np
from sawyer_motion_planner import CasadiKinematics, SawyerPlanner
from tossingbot import config as cfg

def main():
    # 1. Setup URDF path
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    
    BASE_LINK = cfg.BASE_LINK.value
    TIP_LINK = cfg.END_LINK.value
    
    print(f"Loading URDF: {urdf_path}")
    
    # 2. Test 7-DOF Mode (Point-to-Point)
    kin = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
    planner = SawyerPlanner(kin)
    
    print(f"SawyerPlanner initialized. Full DOF: {kin.n_dof}")
    
    q_start = np.array(cfg.NEUTRAL_JOINT_POS)
    q_end = np.array(cfg.TOSS_READY_POS)
    
    print("Solving 7-DOF P2P...")
    traj7 = planner.plan_joint(q_start, q_end, duration=1.0)
    if traj7:
        print(f"Success! Trajectory length: {len(traj7.points)}")
    else:
        print("Failed to solve 7-DOF P2P.")

    # 3. Test Tossing Mode (Uses 3-DOF internally)
    print("\nTesting plan_toss (Internal 3-DOF mapping)...")
    
    # Starting position for all 7 joints
    q_start_full = np.array(cfg.TOSS_READY_POS)
    
    # Get initial position
    initial_pos = kin.fk_pos(q_start_full)
    print(f"Initial Pos: {initial_pos}")

    # Target position
    target_pos = np.array([0.8, 0.0, 0.4]) 
    target_speed = 2.0 
    release_angle = np.deg2rad(45)
    
    print(f"Solving Toss to {target_pos} @ {target_speed} m/s...")
    traj_toss = planner.toss_2d(q_start_full, target_pos[:2], target_speed, 45.0)
    if traj_toss:
        print(f"Success! Tossing trajectory length: {len(traj_toss.points)}")
        last_q = traj_toss.points[-1].position.to_array()
        actual_pos = kin.fk_pos(last_q)
        print(f"Actual End Pos: {actual_pos}")
    else:
        print("Failed to solve Toss.")

if __name__ == "__main__":
    main()
