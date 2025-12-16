# FILE: src/tossing_system/src/tossingbot/planning/casadi_planner.py
import casadi as ca
import numpy as np
import rospy
from scipy.interpolate import interp1d
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.planner_config import PlannerConfig

class CasadiPlanner:
    def __init__(self, kinematics: CasadiKinematics, config: PlannerConfig = None):
        self.model = kinematics
        # Use default config if none provided
        self.cfg = config if config else PlannerConfig()
        
        # Pre-convert Q_NATURAL to CasADi DM for efficiency
        self.Q_NATURAL = ca.DM(self.cfg.q_natural)

    def _setup_problem(self, duration, q_start, check_floor):
        """
        Sets up the common optimization variables, physics constraints, and boundary conditions.
        """
        # 1. Time & Steps
        steps = self.cfg.solver_steps
        dt = duration / steps
        
        opti = ca.Opti()
        
        # 2. Variables
        Q = opti.variable(self.model.n_dof, steps)
        V = opti.variable(self.model.n_dof, steps)
        A = opti.variable(self.model.n_dof, steps)
        SLACK = opti.variable(1, steps)

        prev_q = ca.DM(q_start)
        floor_cost_accum = 0
        
        # 3. Physics & Constraints Loop
        for k in range(steps):
            q_k, v_k, a_k = Q[:,k], V[:,k], A[:,k]
            s_k = SLACK[:,k]
            
            # Integration (Euler)
            if k == 0:
                opti.subject_to(q_k == prev_q + v_k * dt)
                opti.subject_to(v_k == 0) # Start from rest
            else:
                q_prev, v_prev = Q[:,k-1], V[:,k-1]
                opti.subject_to(q_k == q_prev + v_k * dt)
                opti.subject_to(v_k == v_prev + a_k * dt)

            # Robot Limits
            opti.subject_to(opti.bounded(self.model.q_min, q_k, self.model.q_max))
            opti.subject_to(opti.bounded(-self.cfg.max_acc, a_k, self.cfg.max_acc))
            
            # Floor Safety (Soft Constraint)
            if check_floor:
                pos_k = self.model.fk_pos(q_k)
                opti.subject_to(s_k >= 0)
                # Ensure gripper stays above table (minus slack)
                opti.subject_to(pos_k[2] >= self.cfg.table_height - s_k)
                floor_cost_accum += self.cfg.w_slack * (s_k**2)
            else:
                opti.subject_to(s_k == 0)

        # End Condition: Stop at the end
        opti.subject_to(V[:, -1] == 0)
        
        return opti, Q, V, A, floor_cost_accum

    def _solve_and_extract(self, opti, Q, V, A, duration, q_start):
        """
        Runs the solver and upsamples the trajectory.
        """
        # Warm Start (Linear interpolation initialization often helps convergence)
        for k in range(self.cfg.solver_steps):
            opti.set_initial(Q[:, k], q_start)
            
        opts = {
            'ipopt.print_level': 0, 
            'ipopt.sb': 'yes', 
            'ipopt.max_iter': 500, 
            'print_time': 0, 
            'ipopt.max_cpu_time': 2.0
        }
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            
            # Extract raw coarse trajectory
            res_q = sol.value(Q).T 
            res_v = sol.value(V).T
            res_a = sol.value(A).T
            
            return self._upsample_trajectory(res_q, res_v, res_a, duration)
            
        except RuntimeError:
            rospy.logwarn("[CasadiPlanner] Solver failed to converge!")
            return None

    def _upsample_trajectory(self, q, v, a, duration):
        """
        Interpolates the coarse solver output (20-40 steps) into fine motor commands (100Hz).
        """
        steps = self.cfg.solver_steps
        t_coarse = np.linspace(0, duration, steps)
        
        # 100Hz equivalent steps
        n_fine_steps = int(duration * 100)
        t_fine = np.linspace(0, duration, n_fine_steps)
        
        # Vectorized interpolation
        q_fine = interp1d(t_coarse, q, axis=0, kind='cubic')(t_fine)
        v_fine = interp1d(t_coarse, v, axis=0, kind='linear')(t_fine)
        a_fine = interp1d(t_coarse, a, axis=0, kind='linear')(t_fine)
        
        traj = []
        for k in range(n_fine_steps):
            traj.append({
                'position': q_fine[k].tolist(),
                'velocity': v_fine[k].tolist(),
                'acceleration': a_fine[k].tolist()
            })
        return traj

    # ==========================================================================
    # PUBLIC PLANNING METHODS
    # ==========================================================================
    
    def plan_joint(self, q_start, q_goal, duration=3.0, speed_ratio=0.5, check_floor=False):
        opti, Q, V, A, floor_cost = self._setup_problem(duration, q_start, check_floor)
        
        # Velocity Limits scaled by speed_ratio
        limit_v = self.cfg.max_vel * np.clip(speed_ratio, 0.05, 1.0)
        opti.subject_to(opti.bounded(-limit_v, V, limit_v))

        total_cost = floor_cost
        target_q = ca.DM(q_goal)
        prev_q = ca.DM(q_start)
        
        for k in range(self.cfg.solver_steps):
            # 1. Smoothness (minimize jerk/accel)
            total_cost += self.cfg.w_smooth * ca.dot(A[:,k], A[:,k])
            
            # 2. Tracking (Linear interpolation reference)
            alpha = (k + 1) / self.cfg.solver_steps
            ref_q = prev_q * (1 - alpha) + target_q * alpha
            err = Q[:,k] - ref_q
            total_cost += self.cfg.w_track * ca.dot(err, err)

        # 3. Goal Constraint (Soft but strong)
        err_final = Q[:, -1] - target_q
        total_cost += self.cfg.w_goal * ca.dot(err_final, err_final)
        
        opti.minimize(total_cost)
        return self._solve_and_extract(opti, Q, V, A, duration, q_start)

    def plan_cartesian(self, q_start, target_pos, target_quat=[0,1,0,0], duration=3.0, speed_ratio=0.5, check_floor=False):
        opti, Q, V, A, floor_cost = self._setup_problem(duration, q_start, check_floor)
        
        limit_v = self.cfg.max_vel * np.clip(speed_ratio, 0.05, 1.0)
        opti.subject_to(opti.bounded(-limit_v, V, limit_v))

        total_cost = floor_cost
        p0 = np.array(self.model.fk_pos(q_start)).flatten()
        pf = np.array(target_pos)
        q_target = ca.DM(target_quat)
        
        for k in range(self.cfg.solver_steps):
            total_cost += self.cfg.w_smooth * ca.dot(A[:,k], A[:,k])
            
            pos_k = self.model.fk_pos(Q[:,k])
            rot_k = self.model.fk_rot(Q[:,k])
            
            # Linear Position Reference (Straight line in Cartesian space)
            alpha = (k + 1) / self.cfg.solver_steps
            pos_ref = p0 * (1 - alpha) + pf * alpha
            err_pos = pos_k - ca.DM(pos_ref)
            
            total_cost += self.cfg.w_pos * ca.dot(err_pos, err_pos)
            
            # Orientation Error (Dot product maximization)
            dot_prod = ca.dot(rot_k, q_target)
            err_ori = 1.0 - (dot_prod * dot_prod)
            total_cost += self.cfg.w_ori * err_ori

            # Posture Regularization (Fixes "Snaking")
            # Pulls non-essential joints towards a natural home pose
            q_diff = Q[:, k] - self.Q_NATURAL
            total_cost += self.cfg.w_reg * ca.dot(q_diff, q_diff)

        opti.minimize(total_cost)
        return self._solve_and_extract(opti, Q, V, A, duration, q_start)
    
    def compute_inverse_kinematics(self, q_start, target_pos, target_quat=[0,1,0,0]):
        """
        Solves static IK using the same weights/costs as the trajectory planner
        to ensure consistency.
        """
        opti = ca.Opti()
        Q = opti.variable(self.model.n_dof)
        
        p_target = ca.DM(target_pos)
        q_target = ca.DM(target_quat)
        
        pos_curr = self.model.fk_pos(Q)
        rot_curr = self.model.fk_rot(Q)
        
        total_cost = 0
        
        # 1. Position
        err_pos = pos_curr - p_target
        total_cost += self.cfg.w_pos * ca.dot(err_pos, err_pos)
        
        # 2. Orientation
        dot_prod = ca.dot(rot_curr, q_target)
        err_ori = 1.0 - (dot_prod * dot_prod)
        total_cost += self.cfg.w_ori * err_ori
        
        # 3. Regularization (Posture)
        diff_natural = Q - self.Q_NATURAL
        total_cost += self.cfg.w_reg * ca.dot(diff_natural, diff_natural)

        # 4. Stay close to start (Stability bias)
        diff_start = Q - ca.DM(q_start)
        total_cost += 0.1 * ca.dot(diff_start, diff_start)

        opti.subject_to(opti.bounded(self.model.q_min, Q, self.model.q_max))
        opti.minimize(total_cost)
        
        opti.set_initial(Q, q_start)
        opts = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            return sol.value(Q).tolist()
        except RuntimeError:
            return None