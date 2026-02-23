import numpy as np
import casadi as ca
from .config import ROBOT_PARAMS

class RobotKinematics(object):
    def __init__(self):
        # 1. Initialize the symbolic graph ONCE when class is loaded
        self._sym_funcs = self._build_casadi_functions()

    def _build_casadi_functions(self):
        """
        INTERNAL: Defines the math using CasADi symbols. 
        Returns a dictionary of compiled CasADi functions.
        """
        # Load params
        l1 = ROBOT_PARAMS["l1"]
        l2 = ROBOT_PARAMS["l2"]
        l3_total = ROBOT_PARAMS["l3"] + ROBOT_PARAMS["gripper_len"]
        base = ROBOT_PARAMS["base_offset"]

        # --- Symbolic Variables ---
        q_s = ca.SX.sym("q", 3)   # Position
        qd_s = ca.SX.sym("qd", 3) # Velocity
        
        # --- Forward Kinematics Math ---
        t1_s = -q_s[0]
        t2_s = -q_s[0] - q_s[1]
        t3_s = -q_s[0] - q_s[1] - q_s[2]

        x_s = base[0] + l1 * ca.cos(t1_s) + l2 * ca.cos(t2_s) + l3_total * ca.cos(t3_s)
        z_s = base[2] + l1 * ca.sin(t1_s) + l2 * ca.sin(t2_s) + l3_total * ca.sin(t3_s)
        theta_s = t3_s 
        
        pose_s = ca.vertcat(x_s, z_s, theta_s)
        
        # --- Jacobian Math ---
        # J: How pose changes as q changes
        J_s = ca.jacobian(pose_s, q_s)
        
        # --- J_dot * q_dot Math (For Acceleration) ---
        # velocity vector = J * qd
        v_s = ca.mtimes(J_s, qd_s) 
        # J_dot_q_dot = d(v)/dq * qd (Approximation for acceleration constraints)
        Jdqd_s = ca.jtimes(v_s, q_s, qd_s) 

        # --- Create Callable Functions ---
        return {
            "fk": ca.Function("fk", [q_s], [pose_s]),
            "jacobian": ca.Function("jacobian", [q_s], [J_s]),
            "jdot_qdot": ca.Function("jdot_qdot", [q_s, qd_s], [Jdqd_s]),
        }

    # =========================================
    #  NUMERIC INTERFACE (For ROS / Testing)
    # =========================================
    
    def forward_kinematics(self, q):
        """
        Input: List or NumPy Array (3,)
        Output: NumPy Array (3,) [x, z, theta]
        """
        # Explicitly cast to float to prevent symbolic errors
        q_in = np.array(q, dtype=float)
        res = self._sym_funcs["fk"](q_in)
        return np.array(res).flatten()

    def get_jacobian(self, q):
        """
        Input: List or NumPy Array (3,)
        Output: NumPy Array (3, 3)
        """
        q_in = np.array(q, dtype=float)
        res = self._sym_funcs["jacobian"](q_in)
        return np.array(res)

    def inverse_kinematics_analytical(self, pose):
        """
        Pure NumPy Analytical Solution. 
        Fastest and safest method for finding Q from Pose.
        """
        l1 = ROBOT_PARAMS["l1"]
        l2 = ROBOT_PARAMS["l2"]
        l3_total = ROBOT_PARAMS["l3"] + ROBOT_PARAMS["gripper_len"]
        base = ROBOT_PARAMS["base_offset"]

        x, z, t3 = pose
        x_rel = x - base[0]
        z_rel = z - base[2]
        
        x_wrist = x_rel - l3_total * np.cos(t3)
        z_wrist = z_rel - l3_total * np.sin(t3)

        D = (x_wrist**2 + z_wrist**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        
        # Safety for float errors near reach limit
        if np.abs(D) > 1.0001:
             raise ValueError("Target unreachable. D={}".format(D))
        D = np.clip(D, -1.0, 1.0)

        q2 = np.arctan2(-np.sqrt(1 - D**2), D)
        q1 = np.arctan2(z_wrist, x_wrist) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
        q3 = t3 - q1 - q2

        return np.array([-q1, -q2, -q3])

    # =========================================
    #  SYMBOLIC INTERFACE (For CasADi / Opti)
    # =========================================
    
    def get_casadi_funcs(self):
        """
        Returns the dictionary of raw CasADi functions.
        Use this INSIDE the optimization loop.
        
        Returns:
            {
                "fk": Function(q) -> [x, z, theta],
                "jacobian": Function(q) -> [3x3 matrix],
                "jdot_qdot": Function(q, qd) -> [3x1 vector]
            }
        """
        return self._sym_funcs