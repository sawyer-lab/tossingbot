import numpy as np
import casadi as ca
from scipy.interpolate import interp1d
from kinematics import RobotKinematics 
from config import TRAJECTORY_CONFIG

class TossingPlanner:
    def __init__(self, profile="express"):
        self.dt = TRAJECTORY_CONFIG["dt"]
        self.w_vel = TRAJECTORY_CONFIG["weights"]["vel"]
        self.w_accel = TRAJECTORY_CONFIG["weights"]["accel"]
        self.w_pos = TRAJECTORY_CONFIG["weights"]["pos"]
        self.joint_limits = TRAJECTORY_CONFIG["joint_limits"]
        self.vel_limits = self.joint_limits[profile]["vel"]
        self.accel_limits = self.joint_limits[profile]["accel"]
        self.pos_limits = self.joint_limits["pos"]
        self.rk = RobotKinematics()
        
    def solve(self, start_q, target_pos, target_vel_vector, duration):
        """
        Solves the optimization problem.
        target_pos: [x, z, theta] (3R target)
        target_vel_vector: [vx, vy, 0] (Cartesian velocity)
        """
        funcs = self.rk._sym_funcs  
        N = int(duration / self.dt)
        n_joints = 3

        opti = ca.Opti()

        Q = opti.variable(n_joints, N + 1)
        Qd = opti.variable(n_joints, N + 1)
        Qdd = opti.variable(n_joints, N + 1)
        cost = 0
        for k in range(N):
            cost += self.w_vel * ca.sumsqr(Qd[:, k + 1] - Qd[:, k])
            cost += self.w_accel * ca.sumsqr(Qdd[:, k + 1] - Qdd[:, k])
            cost += self.w_pos * ca.sumsqr(Q[:, k + 1] - Q[:, k])

        opti.minimize(cost)

        for k in range(N):
            q_next, qd_next = Q[:, k + 1], Qd[:, k + 1]
            q_curr, qd_curr = Q[:, k], Qd[:, k]
            qdd_curr, qdd_next = Qdd[:, k], Qdd[:, k + 1]
            opti.subject_to(q_next == q_curr + self.dt / 2.0 * (qd_curr + qd_next))
            opti.subject_to(qd_next == qd_curr + self.dt / 2.0 * (qdd_curr + qdd_next))

        opti.subject_to(funcs["fk"](Q[:, -1])[0:2] == target_pos[0:2])
        opti.subject_to(
            (ca.mtimes(funcs["jacobian"](Q[:, -1]), Qd[:, -1])[0:2])
            == target_vel_vector[0:2]
        )

        opti.subject_to(Q[:, 0] == start_q)


        for i in range(n_joints):
            opti.subject_to(opti.bounded(self.pos_limits["min"][i], Q[i, :], self.pos_limits["max"][i]))
            opti.subject_to(opti.bounded(-self.vel_limits[i], Qd[i, :], self.vel_limits[i]))
            opti.subject_to(opti.bounded(-self.accel_limits[i], Qdd[i, :], self.accel_limits[i]))
            
            
        q_initial_guess = start_q
        q_final_guess = self.rk.inverse_kinematics_analytical(target_pos)
        q_guess_traj = self.linspace_arrays(q_initial_guess, q_final_guess, N + 1).T
        opti.set_initial(Q, q_guess_traj)

        opti.solver("ipopt", {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": False})
        sol = opti.solve()
        return {
            "Q": sol.value(Q),
            "Qd": sol.value(Qd),
            "Qdd": sol.value(Qdd),
            "time": np.linspace(0, duration, N + 1),
        }

    def scale_trajectory(self, solution, desired_speed):
        # ... [Insert your scale_solution logic here] ...
        pass

    @staticmethod
    def map_to_7dof(q_3dof, qd_3dof, qdd_3dof, base_angle_j0):
        """
        Maps the 3-DOF planar solution to the 7-DOF robot joints.
        """
        N = q_3dof.shape[1]
        q_full = np.zeros((N, 7))
        qd_full = np.zeros((N, 7))
        qdd_full = np.zeros((N, 7))

        # Map active joints (1, 3, 5) corresponding to planar motion
        q_full[:, 1] = q_3dof[0]
        q_full[:, 3] = q_3dof[1]
        q_full[:, 5] = q_3dof[2]
        
        qd_full[:, 1] = qd_3dof[0]
        qd_full[:, 3] = qd_3dof[1]
        qd_full[:, 5] = qd_3dof[2]
        
        qdd_full[:, 1] = qdd_3dof[0] - (np.pi/2) 
        qdd_full[:, 3] = qdd_3dof[1]
        qdd_full[:, 5] = qdd_3dof[2]

        # Set fixed joints
        q_full[:, 0] = base_angle_j0
        q_full[:, 6] = 1.766 # Fixed wrist orientation

        return q_full, qd_full, qdd_full
    

    @staticmethod
    def linspace_arrays(start, stop, num):
        """
        Vectorized linear interpolation between two arrays.
        """
        start = np.asarray(start, dtype=np.float)
        stop = np.asarray(stop, dtype=np.float)
        
        # 1. Generate the interpolation steps (0.0 to 1.0)
        # Shape: (num,)
        steps = np.linspace(0, 1, num)
        
        # 2. Reshape to (num, 1) to allow broadcasting
        # In Python 2.7 / Old Numpy, use np.newaxis or None
        steps = steps[:, np.newaxis]
        
        # 3. Calculate path (Linear Interpolation Formula: p = A + (B-A)*t)
        # Broadcasting: (num, 1) * (dims,) -> (num, dims)
        return start + (stop - start) * steps