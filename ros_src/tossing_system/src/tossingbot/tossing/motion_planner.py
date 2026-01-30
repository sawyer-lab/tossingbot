import numpy as np
import casadi as ca
from scipy.interpolate import interp1d
from tossingbot.tossing.kinematics import RobotKinematics
from tossingbot.tossing.config import TRAJECTORY_CONFIG
from tossingbot.tossing.cache_utils import SimpleTrajectoryCache
from tossingbot.tossing.plotting import *

class TossingPlanner:
    def __init__(self, profile="express", max_speed = 2.0, angle_deg=45,
                  q0 = None, xT = np.array([0.825, 0.0, 0.0])):
        self.dt = TRAJECTORY_CONFIG["dt"]
        self.w_vel = TRAJECTORY_CONFIG["weights"]["vel"]
        self.w_accel = TRAJECTORY_CONFIG["weights"]["accel"]
        self.w_pos = TRAJECTORY_CONFIG["weights"]["pos"]
        self.joint_limits = TRAJECTORY_CONFIG["joint_limits"]
        self.vel_limits = self.joint_limits[profile]["vel"]
        self.accel_limits = self.joint_limits[profile]["accel"]
        self.pos_limits = self.joint_limits["pos"]
        self.rk = RobotKinematics()
        self.cache = SimpleTrajectoryCache(clear_on_start=True)
        self.max_speed = max_speed
        self.angle = np.deg2rad(angle_deg)  # radians

        if q0 is None:
            # Replicate 3r/exec_refactored.py logic:
            # q0 = inverse_kinematics_3r(np.array([0.175, 0.025, -2.094...]))
            start_pose = np.array([0.175, 0.025, -2.094395102393195])
            self.intialConf = self.rk.inverse_kinematics_analytical(start_pose)
        else:
            self.intialConf = q0

        self.targetPosition = xT
        self.min_duration = 0.7
        self.stop_time = 1.5


    def get_trajectory(self,  target_speed):
        
        if target_speed > self.max_speed:
            raise ValueError("Requested target velocity exceeds max speed of {:.2f} " \
            "m/s".format(self.max_speed))
        

        cached_sol = self.cache.load(target_speed)
        if cached_sol is not None and "index" in cached_sol:
            print("Using cached solution.")
            return cached_sol
        
        # ALWAYS SOLVE FRESH (Bypass scaling to fix swinging)
        # Use min_duration or scale duration based on speed?
        # For now, keep min_duration or let solve handle it.
        # Ideally duration should scale: duration = self.min_duration * (self.max_speed / target_speed)
        # But let's try fixed duration or just rely on solve's N
        
        # Simple duration scaling to avoid asking for impossible accel
        # If 0.7s is for 2.0m/s, then for 1.5m/s we might need more time? 
        # Actually slower speed = more time usually? No, same path?
        # Let's use self.min_duration for now.
        
        sol = self.solve(self.intialConf, self.targetPosition, target_speed, self.min_duration)
        
        index = sol["Q"].shape[1]
        total_sol = self.append_stop_trajectory(
            sol["Q"], sol["Qd"], sol["Qdd"]
        )
        total_sol["index"] = index

        self.cache.save(target_speed, total_sol)
        return total_sol
        
        # --- OLD SCALING LOGIC DISABLED ---
        # if target_speed == self.max_speed:
        #    ...
        

    def solve(self, start_q, target_pos, target_speed, duration):

        target_vel_vector = [target_speed * np.cos(self.angle),
                                       target_speed * np.sin(self.angle), 
                                       0]
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
        q_guess_traj = np.linspace(q_initial_guess, q_final_guess, N + 1).T
        opti.set_initial(Q, q_guess_traj)

        opti.solver("ipopt", {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": False})
        sol = opti.solve()

        solution = {
            "Q": sol.value(Q),
            "Qd": sol.value(Qd),
            "Qdd": sol.value(Qdd),
            "time": np.linspace(0, duration, N + 1)        }

        return solution
    


    def scale_casadi_solution_taskspace_peak(self, solution_max, v_desired, dt=0.01):
        t_old = np.asarray(solution_max["time"])
        Q_old = np.asarray(solution_max["Q"])    # shape (3, N)
        Qd_old = np.asarray(solution_max["Qd"])
        Qdd_old = np.asarray(solution_max["Qdd"])

        # scaling factor: velocities scale by s, accelerations by s^2, time by 1/s
        s = v_desired / self.max_speed
      
        # new total time
        T_old = t_old[-1]
        T_new = T_old / s

        # build uniform time vector at controller dt (include final point)
        t_new = np.arange(0.0, T_new + dt/2, dt)

        # to sample old trajectories at the times corresponding to new timeline:
        # old_time_at_tnew = s * t_new  because t_old = s * t_new  (since t_new = t_old / s)
        t_query = s * t_new
        # ensure query lies within original time domain (tiny epsilon ok)
        t_query = np.clip(t_query, t_old[0], t_old[-1])

        # build interpolators based on old solution (interpolate along time for each column)
        interp_Q   = interp1d(t_old, Q_old.T, axis=0, kind='linear', fill_value='extrapolate')
        interp_Qd  = interp1d(t_old, Qd_old.T, axis=0, kind='linear', fill_value='extrapolate')
        interp_Qdd = interp1d(t_old, Qdd_old.T, axis=0, kind='linear', fill_value='extrapolate')

        Q_at_query   = interp_Q(t_query)   # shape (len(t_new), 3)
        Qd_at_query  = interp_Qd(t_query)
        Qdd_at_query = interp_Qdd(t_query)

        # Now apply scaling to velocities and accelerations
        Q_new   = Q_at_query.T              # shape (3, M)
        Qd_new  = (s * Qd_at_query).T
        Qdd_new = (s**2 * Qdd_at_query).T


        solution_scaled = {"time": t_new, "Q": Q_new, "Qd": Qd_new, "Qdd": Qdd_new}
        return solution_scaled
    
    def refine_trajectory_for_exact_speed(self, scaled_solution, target_speed):

        target_vel_vector = [target_speed * np.cos(self.angle),
                                       target_speed * np.sin(self.angle), 
                                       0]

        # Extract scaled trajectory
        t_scaled = np.asarray(scaled_solution["time"])
        Q_scaled = np.asarray(scaled_solution["Q"])
        Qd_scaled = np.asarray(scaled_solution["Qd"])
        Qdd_scaled = np.asarray(scaled_solution["Qdd"])
        funcs = self.rk._sym_funcs  

        N = t_scaled.size - 1
        n_joints = 3
        
        
        # Setup optimization with scaled trajectory as initial guess
        opti = ca.Opti()
        Q = opti.variable(n_joints, N + 1)
        Qd = opti.variable(n_joints, N + 1)
        Qdd = opti.variable(n_joints, N + 1)
        
        # Set initial guess from scaled solution
        opti.set_initial(Q, Q_scaled)
        opti.set_initial(Qd, Qd_scaled)
        opti.set_initial(Qdd, Qdd_scaled)
        
        # Cost: minimize deviation from scaled trajectory while achieving exact speed
        cost = 0

        for k in range(N):
            # Smoothness terms
            cost += self.w_accel * ca.sumsqr(Qdd[:, k + 1] - Qdd[:, k])
            cost += self.w_pos * ca.sumsqr(Q[:, k + 1] - Q[:, k])
            
            # Penalize deviation from scaled trajectory (soft constraint)
            cost += 0.1 * ca.sumsqr(Q[:, k] - Q_scaled[:, k])
            cost += 0.01 * ca.sumsqr(Qd[:, k] - Qd_scaled[:, k])
        
        opti.minimize(cost)
        
        # Dynamics constraints
        for k in range(N):
            q_next, qd_next = Q[:, k + 1], Qd[:, k + 1]
            q_curr, qd_curr = Q[:, k], Qd[:, k]
            qdd_curr, qdd_next = Qdd[:, k], Qdd[:, k + 1]
            opti.subject_to(q_next == q_curr + self.dt / 2.0 * (qd_curr + qd_next))
            opti.subject_to(qd_next == qd_curr + self.dt / 2.0 * (qdd_curr + qdd_next))
        
        # Boundary conditions
        opti.subject_to(Q[:, 0] == self.intialConf)
        opti.subject_to(funcs["fk"](Q[:, -1])[0:2] == self.targetPosition[0:2])
        
        opti.subject_to(
            (ca.mtimes(funcs["jacobian"](Q[:, -1]), Qd[:, -1])[0:2])
            == target_vel_vector[0:2]
        )
        
        
        for i in range(n_joints):
            opti.subject_to(opti.bounded(self.pos_limits["min"][i], Q[i, :], self.pos_limits["max"][i]))
            opti.subject_to(opti.bounded(-self.vel_limits[i], Qd[i, :], self.vel_limits[i]))
            opti.subject_to(opti.bounded(-self.accel_limits[i], Qdd[i, :], self.accel_limits[i]))
        
        # Solve
        opti.solver("ipopt", {"ipopt.print_level": 0, "print_time": False})
        sol = opti.solve()
        

        refined_solution = {
            "time": t_scaled,
            "Q": sol.value(Q),
            "Qd": sol.value(Qd),
            "Qdd": sol.value(Qdd)
        }
        
        
        return refined_solution
    

    def smooth_stop_segment(self, q_last, qd_last, qdd_last, q_target=None):

        q_initial = q_target if q_target is not None else q_last
       
        N = int(self.stop_time / self.dt)
        n_joints = len(q_last)

        kinematics = self.rk._sym_funcs

        
        opti = ca.Opti()

        # Variables
        q = opti.variable(n_joints, N+1)
        qd = opti.variable(n_joints, N+1)
        qdd = opti.variable(n_joints, N+1)

        # Initial conditions
        opti.subject_to(q[:,0] == q_last)
        opti.subject_to(qd[:,0] == qd_last)
        opti.subject_to(qdd[:,0] == qdd_last)


        # Dynamics constraints (trapezoidal integration)
        for k in range(N):
            opti.subject_to(q[:,k+1] == q[:,k] + self.dt/2.0 * (qd[:,k] + qd[:,k+1]))
            opti.subject_to(qd[:,k+1] == qd[:,k] + self.dt/2.0 * (qdd[:,k] + qdd[:,k+1]))
            
            # Hard constraints on joint limits
            for i in range(n_joints):
                opti.subject_to(opti.bounded(self.pos_limits["min"][i], q[i,k], self.pos_limits["max"][i]))
                opti.subject_to(opti.bounded(-self.accel_limits[i], qdd[i,k], self.accel_limits[i]))

        # Final position within limits (hard constraint)
        for i in range(n_joints):
            opti.subject_to(opti.bounded(self.pos_limits["min"][i], q[i,N], self.pos_limits["max"][i]))

        # Objective: smooth stop while trying to return towards initial position
        cost = 0
        
        # 1. Smoothness (minimize jerk)
        for k in range(N):
            cost += 10.0 * ca.sumsqr(qdd[:,k+1] - qdd[:,k])
        
        # 2. SOFT constraint: encourage near-zero velocities (especially at end)
        for k in range(N):
            cost += 0.5 * ca.sumsqr(qd[:,k])
        cost += 100.0 * ca.sumsqr(qd[:,N])  # Strongly encourage zero final velocity
        cost += 100.0 * ca.sumsqr(qdd[:,N])  # Strongly encourage zero final acceleration
        
        # 3. SOFT constraint: encourage joint velocities within limits
        for k in range(N):
            for i in range(n_joints):
                # Soft penalty if velocity exceeds limit
                vel_violation = ca.fmax(0, ca.fabs(qd[i,k]) - self.vel_limits[i])
                cost += 50.0 * ca.sumsqr(vel_violation)
        
        opti.minimize(cost)

        opti.solver('ipopt', {"print_time": 0}, 
                {"print_level": 0, "max_iter": 2000, "tol": 1e-6, "acceptable_tol": 1e-4})
        
        # Set initial guess (constant position, decaying velocity/accel)
        for i in range(n_joints):
            opti.set_initial(q[i,:], q_last[i])
            opti.set_initial(qd[i,:], np.linspace(qd_last[i], 0, N+1))
            opti.set_initial(qdd[i,:], np.linspace(qdd_last[i], 0, N+1))
        
        try:
            sol = opti.solve()
        except RuntimeError as e:
            # If exact solution fails, try with relaxed tolerances
            print("[stop] Initial solve failed, trying with relaxed constraints...")
            opti.solver('ipopt', {"print_time": 0}, 
                    {"print_level": 0, "max_iter": 3000, "tol": 1e-5, "acceptable_tol": 1e-3, 
                        "constr_viol_tol": 1e-3, "compl_inf_tol": 1e-3})
            sol = opti.solve()

        q_val = np.array(sol.value(q))
        qd_val = np.array(sol.value(qd))
        qdd_val = np.array(sol.value(qdd))
        

        return q_val, qd_val, qdd_val
    
    
    def append_stop_trajectory(self, q_full, qd_full, qdd_full):

        q_last = q_full[:,-1]
        qd_last = qd_full[:,-1]
        qdd_last = qdd_full[:,-1]
        q_initial = q_full[:,0]  # Get initial configuration from trajectory

        q_stop, qd_stop, qdd_stop = self.smooth_stop_segment(
            q_last, qd_last, qdd_last, q_target=q_initial
        )

        # Avoid duplicate at junction
        q_concat = np.concatenate((q_full, q_stop[:,1:]), axis=1)
        qd_concat = np.concatenate((qd_full, qd_stop[:,1:]), axis=1)
        qdd_concat = np.concatenate((qdd_full, qdd_stop[:,1:]), axis=1)

        return {
            "Q": q_concat,
            "Qd": qd_concat,
            "Qdd": qdd_concat,
            "time": np.arange(0, q_concat.shape[1]*self.dt, self.dt)
        }


    @staticmethod
    def map_to_7dof(q_3dof, qd_3dof, qdd_3dof, base_angle_j0):
        """
        Maps the 3-DOF planar solution to the 7-DOF robot joints.
        Returns a dictionary compatible with _execute_trajectory.
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
        
        qdd_full[:, 1] = qdd_3dof[0]
        qdd_full[:, 3] = qdd_3dof[1]
        qdd_full[:, 5] = qdd_3dof[2]

        # Set fixed joints
        q_full[:, 0] = 0.0 # Force X-axis alignment
        q_full[:, 6] = 1.766 # Fixed wrist orientation

        return {
            'Q': q_full,
            'Qd': qd_full,
            'Qdd': qdd_full
        }
    

    # @staticmethod
    # def linspace_arrays(start, stop, num):
    #     """
    #     Vectorized linear interpolation between two arrays.
    #     """
    #     start = np.asarray(start, dtype=np.float)
    #     stop = np.asarray(stop, dtype=np.float)
        
    #     # 1. Generate the interpolation steps (0.0 to 1.0)
    #     # Shape: (num,)
    #     steps = np.linspace(0, 1, num)
        
    #     # 2. Reshape to (num, 1) to allow broadcasting
    #     # In Python 2.7 / Old Numpy, use np.newaxis or None
    #     steps = steps[:, np.newaxis]
        
    #     # 3. Calculate path (Linear Interpolation Formula: p = A + (B-A)*t)
    #     # Broadcasting: (num, 1) * (dims,) -> (num, dims)
    #     return start + (stop - start) * steps