
import os
import numpy as np
import casadi as ca
import time
from sawyer_robot import SawyerRobot, JointAngles
from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics

def run_robot_ik():
    print("--- ROBOT IK EXECUTION TEST ---")
    
    # 1. Initialize Robot Client
    print("Connecting to robot...")
    robot = SawyerRobot(host='localhost', port=5555)
    
    # Ensure robot is enabled
    if not robot.get_robot_status().get('enabled', False):
        print("Enabling robot...")
        robot.enable()
        time.sleep(1.0)

    # 2. Setup Kinematics
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    print(f"Loading URDF: {urdf_path}")
    model = CasadiKinematics(urdf_path, cfg.BASE_LINK, cfg.END_LINK)
    
    # 3. Get Current State and Define Target
    current_joints = robot.arm.get_joints()
    q_curr = [
        current_joints.j0, current_joints.j1, current_joints.j2,
        current_joints.j3, current_joints.j4, current_joints.j5, current_joints.j6
    ]
    
    # Target: 10cm above current position
    start_pos = np.array(model.fk_pos(q_curr)).flatten()
    target_pos = start_pos + np.array([0, 0, 0.10])
    target_quat = np.array(model.fk_rot(q_curr)).flatten() # Maintain current orientation
    
    print(f"Current Position: {np.round(start_pos, 3)}")
    print(f"Target Position:  {np.round(target_pos, 3)}")

    # 4. Solve IK
    q = ca.SX.sym('q', model.n_dof)
    fk_res = model.fk_full(q)
    cost = 10.0 * ca.sumsqr(fk_res[:3] - target_pos) + 1.0 * (1 - ca.power(ca.dot(fk_res[3:], target_quat), 2))
    
    solver = ca.nlpsol('solver', 'ipopt', {'x': q, 'f': cost}, {'ipopt.print_level': 0, 'print_time': 0})
    sol = solver(x0=q_curr, lbx=model.q_min, ubx=model.q_max)
    q_sol = np.array(sol['x']).flatten()

    # 5. Execute Move
    print("Executing move to solved joint angles...")
    
    # Create JointAngles object for the client
    target_angles = JointAngles(
        j0=q_sol[0], j1=q_sol[1], j2=q_sol[2],
        j3=q_sol[3], j4=q_sol[4], j5=q_sol[5], j6=q_sol[6]
    )
    
    success = robot.arm.move(target_angles, timeout=10.0)
    
    if success:
        print("Move COMPLETED successfully.")
        # Verify final position
        new_joints = robot.arm.get_joints()
        q_new = [new_joints.j0, new_joints.j1, new_joints.j2, new_joints.j3, new_joints.j4, new_joints.j5, new_joints.j6]
        final_pos = np.array(model.fk_pos(q_new)).flatten()
        print(f"Final Position: {np.round(final_pos, 3)}")
    else:
        print("Move FAILED or timed out.")

    robot.close()

if __name__ == "__main__":
    try:
        run_robot_ik()
    except Exception as e:
        print(f"Error: {e}")
