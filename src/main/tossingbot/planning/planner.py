import casadi as ca
import numpy as np
from typing import List, Dict, Optional, Tuple
from .kinematics import CasadiKinematics

class UnifiedPlanner:
    def __init__(self, kinematics: CasadiKinematics, dt: float = 0.01):
        self.model = kinematics
        self.dt = dt
        
        # Default limits
        self.max_vel = np.ones(self.model.n_dof) * 2.5
        self.max_acc = np.ones(self.model.n_dof) * 10.0

    def solve_toss(self, q_start: np.ndarray, 
                   target_pos: np.ndarray, 
                   target_speed: float, 
                   release_angle: float,
                   duration: float = 0.7):
        """
        Solves for a tossing trajectory using the full kinematic model.
        target_pos: [x, y, z] in robot frame.
        """
        N = int(duration / self.dt)
        opti = ca.Opti()

        # Variables
        Q = opti.variable(self.model.n_dof, N + 1)
        Qd = opti.variable(self.model.n_dof, N + 1)
        Qdd = opti.variable(self.model.n_dof, N + 1)

        # Cost: Smoothness + regularization
        cost = ca.sumsqr(Qdd) * 0.01 + ca.sumsqr(Qd) * 0.001
        opti.minimize(cost)

        # Physics constraints (Trapezoidal integration)
        for k in range(N):
            opti.subject_to(Q[:, k+1] == Q[:, k] + self.dt/2.0 * (Qd[:, k] + Qd[:, k+1]))
            opti.subject_to(Qd[:, k+1] == Qd[:, k] + self.dt/2.0 * (Qdd[:, k] + Qdd[:, k+1]))

        # Boundary Conditions
        opti.subject_to(Q[:, 0] == q_start)
        
        # Tossing Goal (at the end of duration)
        # 1. Fixed spatial position (x, y, z) - Constrain translation only
        opti.subject_to(self.model.fk_pos(Q[:, -1]) == ca.DM(target_pos))
        
        # 2. Velocity vector [vx, vy, vz]
        v_target = ca.vertcat(
            target_speed * np.cos(release_angle),
            0.0, 
            target_speed * np.sin(release_angle)
        )
        jac_at_release = self.model.jacobian(Q[:, -1])
        v_at_release = ca.mtimes(jac_at_release, Qd[:, -1])
        
        # Constrain X and Z velocity components to match target
        # We ignore Y velocity if J0 is fixed
        opti.subject_to(v_at_release[0] == v_target[0])
        opti.subject_to(v_at_release[2] == v_target[2])

        # Limits
        for i in range(self.model.n_dof):
            opti.subject_to(opti.bounded(self.model.q_min[i], Q[i, :], self.model.q_max[i]))
            opti.subject_to(opti.bounded(-self.max_vel[i], Qd[i, :], self.max_vel[i]))
            opti.subject_to(opti.bounded(-self.max_acc[i], Qdd[i, :], self.max_acc[i]))

        # Initial Guess: Linear interpolation for Q
        # We need a rough IK for the target pos to make a good guess
        # Since we don't have a general IK solver here yet, let's just use start_q as guess
        opti.set_initial(Q, ca.repmat(q_start, 1, N+1))

        # Solve
        opti.solver('ipopt', {"print_time": False, "ipopt.print_level": 0})
        try:
            sol = opti.solve()
            return {
                "Q": sol.value(Q).T,
                "Qd": sol.value(Qd).T,
                "Qdd": sol.value(Qdd).T
            }
        except Exception as e:
            # print(f"Solver failed: {e}")
            return None

    def solve_p2p_joint(self, q_start: np.ndarray, q_end: np.ndarray, duration: float = 1.5, speed_scale: float = 1.0):
        """
        Simple joint-space point-to-point motion.
        """
        N = int(duration / self.dt)
        opti = ca.Opti()

        Q = opti.variable(self.model.n_dof, N + 1)
        Qd = opti.variable(self.model.n_dof, N + 1)
        Qdd = opti.variable(self.model.n_dof, N + 1)

        opti.minimize(ca.sumsqr(Qdd))

        for k in range(N):
            opti.subject_to(Q[:, k+1] == Q[:, k] + self.dt/2.0 * (Qd[:, k] + Qd[:, k+1]))
            opti.subject_to(Qd[:, k+1] == Qd[:, k] + self.dt/2.0 * (Qdd[:, k] + Qdd[:, k+1]))

        opti.subject_to(Q[:, 0] == q_start)
        opti.subject_to(Q[:, -1] == q_end)
        opti.subject_to(Qd[:, 0] == 0)
        opti.subject_to(Qd[:, -1] == 0)

        # Scaled Limits
        scaled_vel = self.max_vel * speed_scale
        scaled_acc = self.max_acc * speed_scale

        for i in range(self.model.n_dof):
            opti.subject_to(opti.bounded(self.model.q_min[i], Q[i, :], self.model.q_max[i]))
            opti.subject_to(opti.bounded(-scaled_vel[i], Qd[i, :], scaled_vel[i]))
            opti.subject_to(opti.bounded(-scaled_acc[i], Qdd[i, :], scaled_acc[i]))

        opti.set_initial(Q, np.linspace(q_start, q_end, N + 1).T)

        opti.solver('ipopt', {"print_time": False, "ipopt.print_level": 0})
        try:
            sol = opti.solve()
            return {
                "Q": sol.value(Q).T,
                "Qd": sol.value(Qd).T,
                "Qdd": sol.value(Qdd).T
            }
        except:
            return None

    def append_stop_trajectory(self, q_release: np.ndarray, qd_release: np.ndarray, stop_duration: float = 1.0):
        """
        Generates a trajectory to bring the robot to a stop after release.
        """
        N = int(stop_duration / self.dt)
        opti = ca.Opti()

        Q = opti.variable(self.model.n_dof, N + 1)
        Qd = opti.variable(self.model.n_dof, N + 1)
        Qdd = opti.variable(self.model.n_dof, N + 1)

        opti.minimize(ca.sumsqr(Qdd))

        for k in range(N):
            opti.subject_to(Q[:, k+1] == Q[:, k] + self.dt/2.0 * (Qd[:, k] + Qd[:, k+1]))
            opti.subject_to(Qd[:, k+1] == Qd[:, k] + self.dt/2.0 * (Qdd[:, k] + Qdd[:, k+1]))

        opti.subject_to(Q[:, 0] == q_release)
        opti.subject_to(Qd[:, 0] == qd_release)
        opti.subject_to(Qd[:, -1] == 0)
        opti.subject_to(Qdd[:, -1] == 0)

        for i in range(self.model.n_dof):
            opti.subject_to(opti.bounded(self.model.q_min[i], Q[i, :], self.model.q_max[i]))
            opti.subject_to(opti.bounded(-self.max_vel[i], Qd[i, :], self.max_vel[i]))
            opti.subject_to(opti.bounded(-self.max_acc[i], Qdd[i, :], self.max_acc[i]))

        opti.solver('ipopt', {"print_time": False, "ipopt.print_level": 0})
        try:
            sol = opti.solve()
            return {
                "Q": sol.value(Q).T,
                "Qd": sol.value(Qd).T,
                "Qdd": sol.value(Qdd).T
            }
        except:
            return None
