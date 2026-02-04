import numpy as np
import casadi as ca
from scipy.interpolate import interp1d
from tossingbot.tossing.kinematics import RobotKinematics
from tossingbot.tossing.config import TRAJECTORY_CONFIG
from tossingbot.tossing.cache_utils import SimpleTrajectoryCache
from tossingbot.tossing.plotting import *

class TossingPlanner:
    def __init__(self, profile="express", max_speed = 2.0, angle_deg=45,
                  q0 = None, xT = np.array([0.70, 0.0, 0.0])):
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
            start_pose = np.array([0.175, 0.025, -2.094395102393195])
            self.intialConf = self.rk.inverse_kinematics_analytical(start_pose)
        else:
            self.intialConf = q0

        self.targetPosition = np.array(xT, dtype=float)
        # Orientation Fix: Enforce -45 degrees pitch at release to avoid collision/friction
        self.targetPosition[2] = -self.angle
        
        self.min_duration = 0.7
        self.stop_time = 1.5


    def get_trajectory(self,  target_speed):
        if target_speed > self.max_speed:
            raise ValueError("Requested target velocity exceeds max speed of {:.2f} m/s".format(self.max_speed))
        
        cached_sol = self.cache.load(target_speed)
        if cached_sol is not None and "index" in cached_sol:
            return cached_sol
        
        sol = self.solve(self.intialConf, self.targetPosition, target_speed, self.min_duration)
        
        index = sol["Q"].shape[1]
        total_sol = self.append_stop_trajectory(sol["Q"], sol["Qd"], sol["Qdd"])
        total_sol["index"] = index

        self.cache.save(target_speed, total_sol)
        return total_sol
        

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

        # Target Constraints at the end of the toss trajectory
        # 1. Position AND Orientation
        opti.subject_to(funcs["fk"](Q[:, -1])[0:3] == target_pos[0:3])
        # 2. Velocity
        opti.subject_to(ca.mtimes(funcs["jacobian"](Q[:, -1]), Qd[:, -1])[0:2] == target_vel_vector[0:2])
        # 3. Start State
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

        return {
            "Q": sol.value(Q),
            "Qd": sol.value(Qd),
            "Qdd": sol.value(Qdd),
            "time": np.linspace(0, duration, N + 1)
        }
    
    def smooth_stop_segment(self, q_last, qd_last, qdd_last, q_target=None):
        q_initial = q_target if q_target is not None else q_last
        N = int(self.stop_time / self.dt)
        n_joints = len(q_last)
        opti = ca.Opti()
        q = opti.variable(n_joints, N+1)
        qd = opti.variable(n_joints, N+1)
        qdd = opti.variable(n_joints, N+1)
        opti.subject_to(q[:,0] == q_last)
        opti.subject_to(qd[:,0] == qd_last)
        opti.subject_to(qdd[:,0] == qdd_last)
        for k in range(N):
            opti.subject_to(q[:,k+1] == q[:,k] + self.dt/2.0 * (qd[:,k] + qd[:,k+1]))
            opti.subject_to(qd[:,k+1] == qd[:,k] + self.dt/2.0 * (qdd[:,k] + qdd[:,k+1]))
            for i in range(n_joints):
                opti.subject_to(opti.bounded(self.pos_limits["min"][i], q[i,k], self.pos_limits["max"][i]))
                opti.subject_to(opti.bounded(-self.accel_limits[i], qdd[i,k], self.accel_limits[i]))
        cost = 0
        for k in range(N):
            cost += 10.0 * ca.sumsqr(qdd[:,k+1] - qdd[:,k])
        for k in range(N):
            cost += 0.5 * ca.sumsqr(qd[:,k])
        cost += 100.0 * ca.sumsqr(qd[:,N])
        cost += 100.0 * ca.sumsqr(qdd[:,N])
        opti.minimize(cost)
        opti.solver('ipopt', {"print_time": 0}, {"print_level": 0, "max_iter": 2000, "tol": 1e-6})
        sol = opti.solve()
        return np.array(sol.value(q)), np.array(sol.value(qd)), np.array(sol.value(qdd))
    
    def append_stop_trajectory(self, q_full, qd_full, qdd_full):
        q_last, qd_last, qdd_last = q_full[:,-1], qd_full[:,-1], qdd_full[:,-1]
        q_stop, qd_stop, qdd_stop = self.smooth_stop_segment(q_last, qd_last, qdd_last, q_target=q_full[:,0])
        q_concat = np.concatenate((q_full, q_stop[:,1:]), axis=1)
        qd_concat = np.concatenate((qd_full, qd_stop[:,1:]), axis=1)
        qdd_concat = np.concatenate((qdd_full, qdd_stop[:,1:]), axis=1)
        return {
            "Q": q_concat, "Qd": qd_concat, "Qdd": qdd_concat,
            "time": np.arange(0, q_concat.shape[1]*self.dt, self.dt)
        }

    @staticmethod
    def calculate_base_alignment(target_pos_robot_frame, gripper_pos_robot_frame=None, tilt_angle_deg=45):
        from scipy.spatial.transform import Rotation as R
        x_target, y_target = target_pos_robot_frame[0], target_pos_robot_frame[1]
        j0_angle = np.arctan2(y_target, x_target)
        angle_to_target_xy = j0_angle
        r_base = R.from_quat([0, 1, 0, 0])
        r_z = R.from_euler('z', angle_to_target_xy, degrees=False)
        r_y = R.from_euler('y', tilt_angle_deg, degrees=True)
        orientation_quat = (r_z * r_y * r_base).as_quat()
        return j0_angle, orientation_quat

    @staticmethod
    def map_to_7dof(q_3dof, qd_3dof, qdd_3dof, base_angle_j0, j2=0.0, j4=0.0, j6=1.766):
        N = q_3dof.shape[1]
        q_full, qd_full, qdd_full = np.zeros((N, 7)), np.zeros((N, 7)), np.zeros((N, 7))
        q_full[:, 1], q_full[:, 3], q_full[:, 5] = q_3dof[0], q_3dof[1], q_3dof[2]
        qd_full[:, 1], qd_full[:, 3], qd_full[:, 5] = qd_3dof[0], qd_3dof[1], qd_3dof[2]
        qdd_full[:, 1], qdd_full[:, 3], qdd_full[:, 5] = qdd_3dof[0], qdd_3dof[1], qdd_3dof[2]
        q_full[:, 0], q_full[:, 2], q_full[:, 4], q_full[:, 6] = base_angle_j0, j2, j4, j6
        return {'Q': q_full, 'Qd': qd_full, 'Qdd': qdd_full}
