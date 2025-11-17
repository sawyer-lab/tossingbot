import numpy as np
from kinematics import inverse_kinematics_3r, forward_kinematics_3r
from scipy.interpolate import interp1d
from config import (
    l1,
    l2,
    l3,
    base_offset_x,
    base_offset_z,
    gripper,
)
import casadi as ca



config = {
    "joint_limits": {
        "pos": {
            "min": np.array([-3.8095, -3.0439, -2.9761]),
            "max": np.array([2.2736, 3.0439, 2.9761]),
        },
        "slow": {
            "vel": np.array([0.2825, 0.415, 0.74]),
            "accel": np.array([1.5, 3.0, 3.0]),
        },
        "medium": {
            "vel": np.array([0.678, 0.996, 1.776]),
            "accel": np.array([2.5, 5.0, 5.0]),
        },
        "fast": {
            "vel": np.array([1.13, 1.66, 2.96]),
            "accel": np.array([5.0, 8.0, 8.0]),
        },
        "express": {
            "vel": np.array([1.13, 1.66, 2.96]),
            "accel": np.array([8.0, 10.0, 12.0]),
        },
    },
    "weights": {
        "accel": 1.0,
        "vel": 1.0,
        "pos": 1.0,
        "vel_dir": 1.0,
    },
    "start_tolerance_x": 0.0,  # A small value, e.g., 10 cm
    "start_tolerance_z": 0.0,
    "setup": "express",
    "dt" : 0.01,
    "release_angle" : 45.0 * (np.pi / 180.0),  # radians
}



def linspace_arrays(start, stop, num):
    start = np.asarray(start, dtype=float)
    stop = np.asarray(stop, dtype=float)
    result = np.empty((num, start.size))
    for i in range(num):
        alpha = float(i) / (num - 1) if num > 1 else 0.0
        result[i] = (1 - alpha) * start + alpha * stop
    return result


def get_kinematics_functions():
    q_s = ca.SX.sym("q", 3)
    qd_s = ca.SX.sym("qd", 3)
    t1_s = -q_s[0]
    t2_s = -q_s[0] - q_s[1]
    t3_s = -q_s[0] - q_s[1] - q_s[2]
    L3_gripper = l3 + gripper
    x_s = (
        base_offset_x
        + l1 * ca.cos(t1_s)
        + l2 * ca.cos(t2_s)
        + L3_gripper * ca.cos(t3_s)
    )
    z_s = (
        base_offset_z
        + l1 * ca.sin(t1_s)
        + l2 * ca.sin(t2_s)
        + L3_gripper * ca.sin(t3_s)
    )
    theta_s = t3_s
    pose_s = ca.vertcat(x_s, z_s, theta_s)
    J_s = ca.jacobian(pose_s, q_s)
    p_dot_s = ca.mtimes(J_s, qd_s)
    Jdqd_s = ca.jtimes(p_dot_s, q_s, qd_s)
    return {
        "fk": ca.Function("fk", [q_s], [pose_s]),
        "jacobian": ca.Function("jacobian", [q_s], [J_s]),
        "jdot_qdot": ca.Function("jdot_qdot", [q_s, qd_s], [Jdqd_s]),
    }
    

def solve_trajectory_problem(v_final, T, q0, xT):
    kinematics = get_kinematics_functions()
    h = config["dt"]
    N = int(T / h)
    n_joints = 3

    opti = ca.Opti()

    Q = opti.variable(n_joints, N + 1)
    Qd = opti.variable(n_joints, N + 1)
    Qdd = opti.variable(n_joints, N + 1)
    cost = 0
    w = config["weights"]
    for k in range(N):
        cost += w["vel"] * ca.sumsqr(Qd[:, k + 1] - Qd[:, k])
        cost += w["accel"] * ca.sumsqr(Qdd[:, k + 1] - Qdd[:, k])
        cost += w["pos"] * ca.sumsqr(Q[:, k + 1] - Q[:, k])

    opti.minimize(cost)

    for k in range(N):
        q_next, qd_next = Q[:, k + 1], Qd[:, k + 1]
        q_curr, qd_curr = Q[:, k], Qd[:, k]
        qdd_curr, qdd_next = Qdd[:, k], Qdd[:, k + 1]
        opti.subject_to(q_next == q_curr + h / 2.0 * (qd_curr + qd_next))
        opti.subject_to(qd_next == qd_curr + h / 2.0 * (qdd_curr + qdd_next))


    # for k in range(N + 1):
    #     opti.subject_to(kinematics["fk"](Q[:, k])[1] >= 0.1)


    opti.subject_to(kinematics["fk"](Q[:, -1])[0:2] == xT[0:2])
    opti.subject_to(
        (ca.mtimes(kinematics["jacobian"](Q[:, -1]), Qd[:, -1])[0:2])
        == v_final[0:2]
    )

    opti.subject_to(Q[:, 0] == q0)

    joint_limits = config["joint_limits"]
    q_vel_limits = joint_limits[config["setup"]]["vel"]
    q_accel_limits = joint_limits[config["setup"]]["accel"]
    pos_max = joint_limits["pos"]["max"]
    pos_min = joint_limits["pos"]["min"]

    for i in range(n_joints):
        opti.subject_to(opti.bounded(pos_min[i], Q[i, :], pos_max[i]))
        opti.subject_to(opti.bounded(-q_vel_limits[i], Qd[i, :], q_vel_limits[i]))
        opti.subject_to(opti.bounded(-q_accel_limits[i], Qdd[i, :], q_accel_limits[i]))
        
        
    q_initial_guess = q0
    q_final_guess = inverse_kinematics_3r(xT)
    q_guess_traj = linspace_arrays(q_initial_guess, q_final_guess, N + 1).T
    opti.set_initial(Q, q_guess_traj)

    opti.solver("ipopt", {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": False})
    sol = opti.solve()
    return {
        "Q": sol.value(Q),
        "Qd": sol.value(Qd),
        "Qdd": sol.value(Qdd),
        "time": np.linspace(0, T, N + 1),
    }




def deg2rad(degrees):
    return degrees * (np.pi / 180)


def assemble_full_trajectory(q, qd, qdd, q0=0.0):

    fixed_values = {
            "j0": q0,
            "j2": 0.0,
            "j4": 0.0,
            "j6": 1.7659902159840577,
        }
    N = q.shape[1]
    q_full = np.zeros((N, 7))
    qd_full = np.zeros((N, 7))
    qdd_full = np.zeros((N, 7))

    q_full[:, 1] = q[0]
    q_full[:, 3] = q[1]
    q_full[:, 5] = q[2]
    qd_full[:, 1] = qd[0]
    qd_full[:, 3] = qd[1]
    qd_full[:, 5] = qd[2]
    qdd_full[:, 1] = qdd[0]
    qdd_full[:, 3] = qdd[1]
    qdd_full[:, 5] = qdd[2]

    for idx, joint in zip([0, 2, 4, 6], ["j0", "j2", "j4", "j6"]):
        val = fixed_values[joint]
        q_full[:, idx] = val
        qd_full[:, idx] = 0.0
        qdd_full[:, idx] = 0.0

    return q_full, qd_full, qdd_full

def scale_solution(solution_max, v_desired, dt = 0.01):
    kinematics = get_kinematics_functions()

    t_old = np.asarray(solution_max["time"])
    q_old = np.asarray(solution_max["Q"])
    qd_old = np.asarray(solution_max["Qd"])
    qdd_old = np.asarray(solution_max["Qdd"])

    N_old = t_old.size
    # ee_speed_old = np.zeros(N_old)
    # for k in range(N_old):
    #     J = kinematics["jacobian"](q_old[:, k]).full()[0:2, :]
    #     # v2 = J @ qd_old[:, k]
    #     v2 = np.dot(J, qd_old[:, k])
    #     ee_speed_old[k] = np.linalg.norm(v2)

    v_peak_old = 2.0 # should be near 2.7 ~ 2.9

    # scale factor
    s = v_desired / v_peak_old

    # new total time
    T_new = t_old[-1] / s
    t_new = np.arange(0, T_new + dt / 2, dt)

    t_query = t_new * s
    t_query = np.clip(t_query, t_old[0], t_old[-1])

    q_interp = interp1d(t_old, q_old.T, axis=0, kind='linear', fill_value="extrapolate")
    qd_interp = interp1d(t_old, qd_old.T, axis=0, kind='linear', fill_value="extrapolate")
    qdd_interp = interp1d(t_old, qdd_old.T, axis=0, kind='linear', fill_value="extrapolate")
    
    q_new = q_interp(t_query).T
    qd_new = qd_interp(t_query).T * s
    qdd_new = qdd_interp(t_query).T * s * s

    # ee_speed_new = np.zeros(t_new.size)
    # for k in range(t_new.size):
    #     J = kinematics["jacobian"](q_new[:, k]).full()[0:2, :]
    #     v2 = np.dot(J, qd_new[:, k])
    #     #v2 = J @ qd_new[:, k]
    #     ee_speed_new[k] = np.linalg.norm(v2)
    # v_peak_new = np.max(ee_speed_new)

    return {"time": t_new, "Q": q_new, "Qd": qd_new, "Qdd": qdd_new}
