import os
import time
import numpy as np
import casadi as ca
from sawyer_robot import SawyerRobot, JointAngles
from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.tossing.kinematics import RobotKinematics

def run_pick_and_toss():
    print("--- OPTIMIZED PICK AND TOSS TEST ---")
    
    # 1. Initialize
    robot = SawyerRobot(host='localhost', port=5555)
    if not robot.get_robot_status().get('enabled', False):
        robot.enable()
        time.sleep(1.0)

    rk_analytical = RobotKinematics()
    model = CasadiKinematics(cfg.SAWYER_PNEUMATIC_URDF, cfg.BASE_LINK, cfg.END_LINK)
    planner = CasadiPlanner(model)
    
    # 2. Winning Config from V2 Search
    # Wind-up: [0.70, 0.20] (Task Space)
    # Release: [0.95, 0.50] (Task Space)
    # Speed:   2.92 m/s
    windup_task = [0.70, 0.20, 0.0]
    release_task = [0.95, 0.0, 0.50]
    toss_speed = 2.92

    try:
        # ======================================================================
        # PHASE 1: THE PICK
        # ======================================================================
        print("\n[PHASE 1] Starting Pick Sequence...")
        robot.gripper.open()
        
        # Pick point (example)
        pick_pos = np.array([0.6, 0.13, -0.22])
        pick_quat = np.array([0, 1, 0, 0])
        
        # Move to Pick
        q_curr = robot.arm.get_joints().to_list()
        q_pick = planner.compute_inverse_kinematics(q_curr, pick_pos, pick_quat)
        if q_pick is not None:
            robot.arm.move(JointAngles(*q_pick))
            
        robot.gripper.close()
        time.sleep(1.0)

        # ======================================================================
        # PHASE 2: OPTIMIZED WIND-UP
        # ======================================================================
        print(f"\n[PHASE 2] Moving to Optimized Wind-up: {windup_task[:2]}")
        
        # Calculate 3-DOF subspace joints for the wind-up
        q_windup_3dof = rk_analytical.inverse_kinematics_analytical(windup_task)
        
        # Map to 7-DOF (assuming J0=0, J2=0, J4=0, J6=1.766)
        q_windup_7dof = [0.0, q_windup_3dof[0], 0.0, q_windup_3dof[1], 0.0, q_windup_3dof[2], 1.766]
        
        robot.arm.move(JointAngles(*q_windup_7dof))
        time.sleep(1.0)

        # ======================================================================
        # PHASE 3: THE TOSS
        # ======================================================================
        print(f"\n[PHASE 3] Executing Optimized Toss at {toss_speed} m/s")
        
        # Get Fresh State
        q_curr_full = np.array(robot.arm.get_joints().to_list())
        q0_3dof = np.array([q_curr_full[1], q_curr_full[3], q_curr_full[5]])
        
        # Plan with optimized target
        toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof, xT=release_task)
        sol_3d = toss_planner.get_trajectory(toss_speed)
        sol_7d = toss_planner.map_to_7dof(
            sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
            q_curr_full[0], q_curr_full[2], q_curr_full[4], q_curr_full[6]
        )
        
        peak_idx = sol_3d["index"]
        print(f"  Toss planned. Release at index {peak_idx}")
        
        # Execute
        success = robot._client.execute_stream_trajectory(
            Q=sol_7d['Q'],
            Qd=sol_7d['Qd'],
            Qdd=sol_7d['Qdd'],
            release_index=peak_idx
        )
        
        if success:
            print("\n[SUCCESS] Optimized Pick and Toss complete.")
        else:
            print("\n[ERROR] Trajectory failed.")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.close()

if __name__ == "__main__":
    run_pick_and_toss()
