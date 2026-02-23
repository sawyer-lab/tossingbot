
import os
import time
import numpy as np
import casadi as ca
from sawyer_robot import SawyerRobot, JointAngles
from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.tossing.motion_planner import TossingPlanner

def run_pick_and_toss():
    print("--- PICK AND TOSS INTEGRATION TEST ---")
    
    # 1. Initialize Robot Client (ZMQ)
    print("Connecting to robot...")
    robot = SawyerRobot(host='localhost', port=5555)
    
    # Ensure robot is enabled
    status = robot.get_robot_status()
    if not status.get('enabled', False):
        print("Enabling robot...")
        robot.enable()
        time.sleep(1.0)

    # 2. Setup Planning
    # Use Tabletop Pneumatic URDF
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    print(f"Loading URDF: {urdf_path}")
    model = CasadiKinematics(urdf_path, cfg.BASE_LINK, cfg.END_LINK)
    planner = CasadiPlanner(model)
    
    # 3. Define Pick Position (Assumes object is there)
    # Target: center of workspace, table height + cube half
    pick_pos = np.array([0.6, 0.13, -0.22]) # Table is at -0.25 approx in robot frame
    pick_quat = np.array([0, 1, 0, 0]) # Pointing down
    
    hover_pos = pick_pos + np.array([0, 0, 0.10])

    try:
        # ======================================================================
        # PHASE 1: THE PICK
        # ======================================================================
        print("[PHASE 1] Starting Pick Sequence...")
        
        # Open Gripper
        print("Opening gripper...")
        robot.gripper.open()
        time.sleep(0.5)
        
        # Move to Hover
        print(f"Moving to Hover: {hover_pos}")
        q_curr = robot.arm.get_joints().to_list()
        q_hover = planner.plan_joint(q_curr, planner.compute_inverse_kinematics(q_curr, hover_pos, pick_quat))
        if q_hover:
            # We take the last waypoint for simplicity in this demo move
            target_q = q_hover[-1]['position']
            robot.arm.move(JointAngles(*target_q))
        
        # Descent
        print("Descending to object...")
        q_curr = robot.arm.get_joints().to_list()
        q_pick = planner.plan_cartesian(q_curr, pick_pos, pick_quat, duration=1.0)
        if q_pick:
            target_q = q_pick[-1]['position']
            robot.arm.move(JointAngles(*target_q))
            
        # Grasp
        print("Closing gripper...")
        robot.gripper.close()
        time.sleep(1.0)
        
        # Lift
        print("Lifting...")
        q_curr = robot.arm.get_joints().to_list()
        q_lift = planner.plan_cartesian(q_curr, hover_pos, pick_quat, duration=1.0)
        if q_lift:
            target_q = q_lift[-1]['position']
            robot.arm.move(JointAngles(*target_q))

        # ======================================================================
        # PHASE 2: THE TOSS
        # ======================================================================
        print("[PHASE 2] Starting Toss Sequence...")
        
        # Move to Toss Ready
        print("Moving to Toss Ready position...")
        robot.arm.move(JointAngles(*cfg.TOSS_READY_POS))
        time.sleep(0.5)
        
        # Plan Tossing Trajectory (Simplified 1.5 m/s toss)
        q_curr = np.array(robot.arm.get_joints().to_list())
        q0_3dof = np.array([q_curr[1], q_curr[3], q_curr[5]]) # J1, J3, J5
        toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
        
        speed = 1.5
        sol_3d = toss_planner.get_trajectory(speed)
        sol_7d = toss_planner.map_to_7dof(
            sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
            q_curr[0], q_curr[2], q_curr[4], q_curr[6]
        )
        
        peak_idx = sol_3d["index"]
        print(f"Executing toss at {speed} m/s (Release at index {peak_idx})")
        
        # Execute Stream Trajectory with Async Release
        # Using the correct ZMQ client method 'execute_stream_trajectory'
        success = robot._client.execute_stream_trajectory(
            Q=sol_7d['Q'],
            Qd=sol_7d['Qd'],
            Qdd=sol_7d['Qdd'],
            release_index=peak_idx
        )
        
        if success:
            print("\n[SUCCESS] Pick and Toss cycle complete.")
        else:
            print("\n[ERROR] Trajectory execution failed.")
        
        # Return to Neutral
        print("Returning to Neutral...")
        robot.arm.move(JointAngles(*cfg.NEUTRAL_JOINT_POS))

    except Exception as e:
        print(f"Error during execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.close()

if __name__ == "__main__":
    run_pick_and_toss()
