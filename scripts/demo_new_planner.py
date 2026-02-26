import os
import time
import numpy as np
from sawyer_robot import SawyerRobot
from sawyer_common.geometry import JointAngles, JointTrajectory, Point2D
from sawyer_motion_planner import CasadiKinematics, SawyerPlanner
from tossingbot import config as cfg

def main():
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    BASE_LINK = cfg.BASE_LINK.value
    TIP_LINK = cfg.END_LINK.value
    
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to robot at {host}...")
    
    with SawyerRobot(host=host) as robot:
        print("Enabling robot...")
        robot.enable()
        robot.gripper.close()
        time.sleep(1.0)
        
        # --- INITIALIZE UNIFIED PLANNER ---
        kin = CasadiKinematics(urdf_path, BASE_LINK, TIP_LINK)
        planner = SawyerPlanner(kin)
        planner.set_profile("medium")

        # --- PHASE 1: 7-DOF P2P TO READY POSITION ---
        print("\n--- Phase 1: Moving to Ready Position (7-DOF) ---")
        curr_q = robot.arm.get_joints().to_array()
        ready_q = np.array(cfg.TOSS_READY_POS)
        
        print(f"Solving P2P from current to Ready...")
        p2p_traj = planner.plan_joint(curr_q, ready_q, duration=1.5)
        
        if p2p_traj:
            print("Executing P2P move...")
            robot.arm.stream_trajectory(
                p2p_traj.to_position_array(), 
                p2p_traj.to_velocity_array(), 
                p2p_traj.to_acceleration_array()
            )
            time.sleep(1.5)
        else:
            print("P2P solver failed!")
            return

        # --- PHASE 2: TOSSING ---
        print("\n--- Phase 2: Executing Optimized Toss ---")
        # Use express profile for tossing
        planner.set_profile("express")
        
        curr_q7 = robot.arm.get_joints().to_array()
        
        # Target position in Task Space (Dist, Height)
        # We start from a wind-up position and release at a farther point
        start_task = Point2D(0.6, 0.2)
        release_task = Point2D(0.85, 0.4)
        target_speed = 3.5 # m/s
        
        print(f"Solving Toss from {start_task} to {release_task} @ {target_speed} m/s...")
        toss_traj = planner.toss_2d(
            q_start_full=curr_q7, 
            start_xy=start_task,
            release_xy=release_task, 
            speed=target_speed, 
            theta=45.0
        )
        
        if toss_traj:
            # New planner handles the stop trajectory or returns a full motion
            # Here we stream the result. 
            # In a real scenario, we might want to release the gripper at the end of toss_traj
            release_idx = len(toss_traj.points) - 5 # Simple heuristic for release timing
            
            print(f"Executing Toss with release near index {release_idx}...")
            robot.arm.stream_trajectory(
                toss_traj.to_position_array(),
                toss_traj.to_velocity_array(),
                toss_traj.to_acceleration_array(),
                release_index=release_idx
            )
            print("Toss complete.")
        else:
            print("Toss solver failed! Check reachability for target_pos.")

if __name__ == "__main__":
    main()
