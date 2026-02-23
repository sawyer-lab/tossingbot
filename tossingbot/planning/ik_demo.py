
import os
import sys
import numpy as np
import casadi as ca

from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics

def run_ik_demo():
    print("--- CASADI INVERSE KINEMATICS DEMO ---")
    
    # 1. Setup Kinematics with the new Tabletop Pneumatic URDF
    urdf_path = cfg.SAWYER_PNEUMATIC_URDF
    if not os.path.exists(urdf_path):
        # Fallback if config isn't fully updated or relative paths differ
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.abspath(os.path.join(script_dir, "../../assets/urdf/sawyer_tabletop_pneumatic.urdf"))

    print(f"Loading URDF: {urdf_path}")
    model = CasadiKinematics(urdf_path, cfg.BASE_LINK, cfg.END_LINK)
    
    # 2. Define Target Pose
    # Example: Center of workspace, slightly above table
    target_pos = np.array([0.6, 0.13, -0.1]) 
    target_quat = np.array([0, 1, 0, 0]) # Pointing down [x, y, z, w]
    
    print(f"Target Position: {target_pos}")
    print(f"Target Quaternion: {target_quat}")

    # 3. Formulate IK Problem using CasADi
    # We use the internal CasADi functions from CasadiKinematics
    q = ca.SX.sym('q', model.n_dof)
    fk_res = model.fk_full(q)
    pos_res = fk_res[:3]
    quat_res = fk_res[3:]
    
    # Cost: Distance to target position + orientation alignment
    cost_pos = ca.sumsqr(pos_res - target_pos)
    # Orientation cost (1 - dot_product^2 is a common way to align quaternions)
    cost_ori = 1 - ca.power(ca.dot(quat_res, target_quat), 2)
    
    # Soft regularization to keep it near neutral
    q_neutral = ca.DM(cfg.NEUTRAL_JOINT_POS)
    cost_reg = 1e-4 * ca.sumsqr(q - q_neutral)
    
    total_cost = 10.0 * cost_pos + 1.0 * cost_ori + cost_reg
    
    # Solver Setup
    nlp = {'x': q, 'f': total_cost}
    # Basic IPOPT solver
    opts = {'ipopt.print_level': 0, 'print_time': 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
    
    # 4. Solve
    print("Solving IK...")
    sol = solver(x0=cfg.NEUTRAL_JOINT_POS, lbx=model.q_min, ubx=model.q_max)
    q_sol = np.array(sol['x']).flatten()
    
    # 5. Verify Results
    final_fk = np.array(model.fk_full(q_sol)).flatten()
    final_pos = final_fk[:3]
    final_quat = final_fk[3:]
    
    print("\n--- RESULTS ---")
    print(f"Solved Joint Angles: \n{np.round(q_sol, 4)}")
    print(f"Final Position:      {np.round(final_pos, 4)}")
    print(f"Position Error:      {np.linalg.norm(final_pos - target_pos):.6f} m")
    print(f"Final Quaternion:    {np.round(final_quat, 4)}")
    
    # Calculate orientation error (angular distance)
    dot = np.abs(np.dot(final_quat, target_quat))
    angle_err = 2 * np.arccos(np.clip(dot, 0, 1))
    print(f"Orientation Error:   {np.degrees(angle_err):.4f} degrees")

if __name__ == "__main__":
    try:
        run_ik_demo()
    except Exception as e:
        print(f"Error: {e}")
