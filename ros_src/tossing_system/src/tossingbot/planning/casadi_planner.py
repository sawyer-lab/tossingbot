import casadi as ca
import numpy as np
import rospy
from scipy.interpolate import interp1d # Standard in ROS desktop full
from tossingbot.planning.kinematics import CasadiKinematics

class CasadiPlanner:
    def __init__(self, kinematics: CasadiKinematics):
        self.model = kinematics
        
        self.TABLE_HEIGHT = 0.0
        self.MAX_VEL = np.array([1.7, 1.7, 1.7, 2.0, 2.0, 3.0, 3.0])
        self.MAX_ACC = np.array([2.5, 2.5, 2.5, 3.0, 3.0, 4.0, 4.0])

        # --- OPTIMIZATION CONFIG ---
        # We solve for a fixed number of "Knots" regardless of duration.
        # 40 steps is enough to describe complex curves without checking every millimeter.
        self.SOLVER_STEPS = 40 
        
        self.W_SMOOTH = 0.1
        self.W_SLACK  = 10000.0

    def _setup_problem(self, duration, q_start, check_floor):
        # 1. Calculate Low-Frequency dt
        # If duration is 4.0s, dt becomes 0.1s (10Hz planning)
        dt = duration / self.SOLVER_STEPS
        
        opti = ca.Opti()
        
        # Variables (Coarse)
        Q = opti.variable(self.model.n_dof, self.SOLVER_STEPS)
        V = opti.variable(self.model.n_dof, self.SOLVER_STEPS)
        A = opti.variable(self.model.n_dof, self.SOLVER_STEPS)
        SLACK = opti.variable(1, self.SOLVER_STEPS)

        prev_q = ca.DM(q_start)
        floor_cost_accum = 0
        
        for k in range(self.SOLVER_STEPS):
            q_k, v_k, a_k = Q[:,k], V[:,k], A[:,k]
            s_k = SLACK[:,k]
            
            # Physics Integration
            if k == 0:
                opti.subject_to(q_k == prev_q + v_k * dt)
                opti.subject_to(v_k == 0)
            else:
                q_prev, v_prev = Q[:,k-1], V[:,k-1]
                opti.subject_to(q_k == q_prev + v_k * dt)
                opti.subject_to(v_k == v_prev + a_k * dt)

            # Limits
            opti.subject_to(opti.bounded(self.model.q_min, q_k, self.model.q_max))
            opti.subject_to(opti.bounded(-self.MAX_ACC, a_k, self.MAX_ACC))
            
            # Floor Logic
            if check_floor:
                pos_k = self.model.fk_pos(q_k)
                opti.subject_to(s_k >= 0)
                opti.subject_to(pos_k[2] >= self.TABLE_HEIGHT - s_k)
                floor_cost_accum += self.W_SLACK * (s_k**2)
            else:
                opti.subject_to(s_k == 0)

        opti.subject_to(V[:, -1] == 0)
        
        return opti, Q, V, A, floor_cost_accum, self.SOLVER_STEPS

    def _solve_and_extract(self, opti, Q, V, A, duration, q_start):
        """
        Solves the coarse problem, then upsamples to 100Hz.
        """
        # Warm Start
        for k in range(self.SOLVER_STEPS):
            opti.set_initial(Q[:, k], q_start)
            
        opts = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'ipopt.max_iter': 500}
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            
            # 1. Get Coarse Solution
            coarse_q = sol.value(Q).T # Shape (40, 7)
            coarse_v = sol.value(V).T
            coarse_a = sol.value(A).T
            
            # 2. Upsample to 100Hz (Hardware Rate)
            return self._upsample_trajectory(coarse_q, coarse_v, coarse_a, duration)
            
        except RuntimeError:
            return None

    def _upsample_trajectory(self, q, v, a, duration):
        """
        Interpolates the coarse solver output (e.g. 10Hz) to hardware rate (100Hz).
        """
        # Original Time Grid (e.g. 0.0, 0.1, 0.2 ... 4.0)
        t_coarse = np.linspace(0, duration, self.SOLVER_STEPS)
        
        # Target Time Grid (e.g. 0.00, 0.01, 0.02 ... 4.0)
        n_fine_steps = int(duration * 100)
        t_fine = np.linspace(0, duration, n_fine_steps)
        
        # Linear Interpolation
        # axis=0 means we interpolate along time (rows)
        q_fine = interp1d(t_coarse, q, axis=0, kind='linear')(t_fine)
        v_fine = interp1d(t_coarse, v, axis=0, kind='linear')(t_fine)
        a_fine = interp1d(t_coarse, a, axis=0, kind='linear')(t_fine)
        
        # Package for Interface
        traj = []
        for k in range(n_fine_steps):
            traj.append({
                'position': q_fine[k].tolist(),
                'velocity': v_fine[k].tolist(),
                'acceleration': a_fine[k].tolist()
            })
        return traj

    # ==========================================================================
    # PUBLIC METHODS (API Unchanged)
    # ==========================================================================
    
    def plan_joint(self, q_start, q_goal, duration=3.0, speed_ratio=0.5, check_floor=True):
        opti, Q, V, A, floor_cost, _ = self._setup_problem(duration, q_start, check_floor)
        
        limit_v = self.MAX_VEL * np.clip(speed_ratio, 0.05, 1.0)
        opti.subject_to(opti.bounded(-limit_v, V, limit_v))

        total_cost = floor_cost
        target_q = ca.DM(q_goal)
        prev_q = ca.DM(q_start)
        
        W_TRACK = 10.0
        W_GOAL  = 1000.0

        for k in range(self.SOLVER_STEPS):
            total_cost += self.W_SMOOTH * ca.dot(A[:,k], A[:,k])
            
            # Linear Reference in Joint Space
            alpha = (k + 1) / self.SOLVER_STEPS
            ref_q = prev_q * (1 - alpha) + target_q * alpha
            err = Q[:,k] - ref_q
            total_cost += W_TRACK * ca.dot(err, err)

        err_final = Q[:, -1] - target_q
        total_cost += W_GOAL * ca.dot(err_final, err_final)
        
        opti.minimize(total_cost)
        return self._solve_and_extract(opti, Q, V, A, duration, q_start)

    def plan_cartesian(self, q_start, target_pos, target_quat=[0,1,0,0], duration=3.0, speed_ratio=0.5, check_floor=True):
        opti, Q, V, A, floor_cost, _ = self._setup_problem(duration, q_start, check_floor)
        
        limit_v = self.MAX_VEL * np.clip(speed_ratio, 0.05, 1.0)
        opti.subject_to(opti.bounded(-limit_v, V, limit_v))

        total_cost = floor_cost
        p0 = np.array(self.model.fk_pos(q_start)).flatten()
        pf = np.array(target_pos)
        q_target = ca.DM(target_quat)
        
        W_POS = 1000.0
        W_ORI = 10.0

        for k in range(self.SOLVER_STEPS):
            total_cost += self.W_SMOOTH * ca.dot(A[:,k], A[:,k])
            
            pos_k = self.model.fk_pos(Q[:,k])
            rot_k = self.model.fk_rot(Q[:,k])
            
            alpha = (k + 1) / self.SOLVER_STEPS
            pos_ref = p0 * (1 - alpha) + pf * alpha
            err_pos = pos_k - ca.DM(pos_ref)
            total_cost += W_POS * ca.dot(err_pos, err_pos)
            
            dot_prod = ca.dot(rot_k, q_target)
            err_ori = 1.0 - (dot_prod * dot_prod)
            total_cost += W_ORI * err_ori

        opti.minimize(total_cost)
        return self._solve_and_extract(opti, Q, V, A, duration, q_start)