#!/usr/bin/env python3.8
import numpy as np
import rospkg
from tossingbot.planning.kinematics import CasadiKinematics

def test_fk():
    print("--- TESTING KINEMATICS ---")
    
    # 1. Load URDF
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('tossingbot_system') + "/src/tossingbot/planning/sawyer_electric.urdf"
    model = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    
    print(f"Model Loaded. DOF: {model.n_dof}")
    
    # 2. Test Zero Configuration
    q_zero = [0.0] * 7
    pos = model.fk_pos(q_zero)
    rot = model.fk_rot(q_zero)
    
    print(f"\nFK at Zero Config:")
    print(f"Pos (xyz): {np.round(pos.full().flatten(), 3)}")
    print(f"Rot (quat): {np.round(rot.full().flatten(), 3)}")
    
    # 3. Test Random Config (Sanity Check)
    q_rand = np.random.uniform(-1.0, 1.0, 7)
    pos_r = model.fk_pos(q_rand)
    print(f"\nFK at Random Config: {np.round(q_rand, 2)}")
    print(f"Pos: {np.round(pos_r.full().flatten(), 3)}")
    
    print("\n[PASS] Math compiles successfully.")

if __name__ == "__main__":
    test_fk()