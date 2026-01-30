# FILE: src/tossing_system/src/tossingbot/planning/casadi_planner.py
import casadi as ca
import numpy as np
import rospy
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.planner_config import PlannerConfig

class CasadiPlanner:
    def __init__(self, kinematics: CasadiKinematics, config: PlannerConfig = None):
        self.model = kinematics
        # Use default config if none provided
        self.cfg = config if config else PlannerConfig()
        
        # Pre-convert Q_NATURAL to CasADi DM for efficiency
        self.Q_NATURAL = ca.DM(self.cfg.q_natural)
    
    def _slerp_quaternions(self, q_start, q_target, steps):
        """
        Spherical linear interpolation between two quaternions.
        Args:
            q_start: Starting quaternion [x, y, z, w] (numpy array or list)
            q_target: Target quaternion [x, y, z, w] (numpy array or list)
            steps: Number of interpolation steps
        Returns:
            List of interpolated quaternions as CasADi DM objects
        """
        # Convert to [x, y, z, w] format for scipy
        q_start_np = np.array(q_start)
        q_target_np = np.array(q_target)
        
        # Create Rotation objects (scipy uses [x, y, z, w] internally)
        rot_start = R.from_quat(q_start_np)
        rot_target = R.from_quat(q_target_np)
        
        # Create SLERP interpolator
        key_times = [0, 1]
        key_rots = R.from_quat([q_start_np, q_target_np])
        slerp = Slerp(key_times, key_rots)
        
        # Generate interpolated quaternions
        alphas = np.linspace(0, 1, steps)
        interpolated_rots = slerp(alphas)
        interpolated_quats = interpolated_rots.as_quat()  # Returns [x, y, z, w] format
        
        # Convert to list of CasADi DM objects
        return [ca.DM(q) for q in interpolated_quats]

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
            'ipopt.max_iter': 500,  # Back to original
            'print_time': 0, 
            'ipopt.max_cpu_time': 2.0  # Back to original
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
    
    def plan_joint(self, q_start, q_goal, duration=None, speed_ratio=0.5, check_floor=False, joint_speed=None):
        
        # Calculate duration from speed if not provided
        if duration is None:
            if joint_speed is None: joint_speed = 0.5 # Default 0.5 rad/s
            
            # Max Joint Displacement
            max_diff = np.max(np.abs(np.array(q_goal) - np.array(q_start)))
            
            # Calculate time (add 0.8s buffer for accel/decel)
            duration = (max_diff / joint_speed) + 0.8
            
            # Safety clamp (min 0.5s)
            duration = max(duration, 0.5)

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

        # 3. Goal Constraint (Hard)
        opti.subject_to(Q[:, -1] == target_q)
        opti.subject_to(Q[:, 0] == prev_q)
        
        opti.minimize(total_cost)
        return self._solve_and_extract(opti, Q, V, A, duration, q_start)

    def plan_cartesian(self, q_start, target_pos, target_quat=[0,1,0,0], duration=None, speed_ratio=0.5, check_floor=False, linear_speed=None):
        
        # Calculate duration from speed if not provided
        if duration is None:
            if linear_speed is None: linear_speed = 0.2 # Default 0.2 m/s
            
            # Cartesian Distance
            current_pos = np.array(self.model.fk_pos(q_start)).flatten()
            dist = np.linalg.norm(np.array(target_pos) - current_pos)
            
            # Calculate time (add 0.8s buffer for accel/decel)
            duration = (dist / linear_speed) + 0.8
            
            # Safety clamp (min 0.5s to avoid singularities/extreme accels on tiny moves)
            duration = max(duration, 0.5)

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
            
            # Orientation Error - just track target at every step (no SLERP)
            dot_prod = ca.dot(rot_k, q_target)
            err_ori = 1.0 - (dot_prod * dot_prod)
            total_cost += self.cfg.w_ori * err_ori

            # Posture Regularization
            q_diff = Q[:, k] - self.Q_NATURAL
            total_cost += self.cfg.w_reg * ca.dot(q_diff, q_diff)

        # Final step: Hard constraint to ensure we actually reach the target
        pos_final = self.model.fk_pos(Q[:, -1])
        rot_final = self.model.fk_rot(Q[:, -1])
        
        # Hard constraint on position
        opti.subject_to(pos_final == ca.DM(target_pos))
        
        # Hard constraint on orientation (dot product close to +/- 1)
        dot_prod_final = ca.dot(rot_final, q_target)
        opti.subject_to(dot_prod_final * dot_prod_final >= 0.999)

        # Hard constraint on initial position
        opti.subject_to(Q[:, 0] == ca.DM(q_start))

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