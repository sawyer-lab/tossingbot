#!/usr/bin/env python3
"""
toss_recorder.py — Recording layer for toss experiments.

Provides:
  - _sub_record: daemon thread that drains the 100 Hz PUB socket
  - execute_toss_with_comparison: executes a toss trajectory and collects
    all robot/object state data needed for later analysis and plotting.
"""
import threading
import time
import numpy as np


def _sub_record(host, port, out, stop_ev):
    """Drain the 100Hz PUB socket into out[] until stop_ev is set."""
    import zmq as _zmq
    ctx = _zmq.Context()
    sub = ctx.socket(_zmq.SUB)
    sub.connect(f"tcp://{host}:{port}")
    sub.setsockopt_string(_zmq.SUBSCRIBE, "")
    sub.setsockopt(_zmq.RCVTIMEO, 20)   # 20 ms poll timeout
    while not stop_ev.is_set():
        try:
            out.append(sub.recv_json())
        except _zmq.Again:
            pass
    sub.close()
    ctx.term()


def execute_toss_with_comparison(
    robot, gazebo, contact, sol_7d, release_delay,
    container_host="localhost", object_name="toss_cube",
    kinematics=None,
):
    """
    Execute toss and collect all data needed for comparison plots.

    Args:
        robot: RobotClient instance
        gazebo: GazeboClient instance
        contact: ContactSensorClient instance
        sol_7d: 7-DOF trajectory solution dict with keys Q, Qd, Qdd
        release_delay: Time in seconds from trajectory start to open gripper
        container_host: Hostname of the ZMQ container (default "localhost")
        object_name: Gazebo model name of the thrown object
        kinematics: CasadiKinematics instance for computing commanded EE
                    velocities (optional)

    Returns a results dict:
      planned_q        (N, 7) numpy array
      planned_qd       (N, 7) numpy array
      commanded_ee_vel (N, 3) numpy array or None
      release_index    int
      actual_states    list of dicts from PUB socket (aligned to trajectory)
      obj_traj         list of {t, pos, vel?} from Gazebo during/after flight
      landing          dict from contact sensor or None
    """
    Q, Qd, Qdd = sol_7d["Q"], sol_7d["Qd"], sol_7d["Qdd"]
    N = Q.shape[0]

    release_index = int(release_delay * 100)
    release_index = max(0, min(release_index, N - 1))
    print(f"[INFO] Release at step {release_index}/{N} "
          f"(t={release_index * 0.01:.3f}s)")

    # Compute commanded EE velocities if kinematics provided
    commanded_ee_vel = None
    if kinematics is not None:
        try:
            commanded_ee_vel = np.zeros((N, 3))
            for i in range(N):
                J = kinematics.compute_jacobian(Q[i, :])
                ee_twist = J @ Qd[i, :]
                commanded_ee_vel[i, :] = ee_twist[:3]
        except Exception as e:
            print(f"[WARN] Could not compute commanded EE velocity: {e}")
            commanded_ee_vel = None

    # Start contact sensor detection
    contact.clear()
    contact.start_detection()

    # Start SUB thread before sending trajectory so we don't miss early frames
    actual_states = []
    stop_ev = threading.Event()
    sub_thread = threading.Thread(
        target=_sub_record,
        args=(container_host, 5556, actual_states, stop_ev),
        daemon=True,
    )
    sub_thread.start()

    # REQ socket blocks for N * 0.01 s; SUB thread records in parallel
    robot.execute_toss_trajectory(Q, Qd, Qdd, release_index)

    stop_ev.set()
    sub_thread.join(timeout=0.5)

    # Keep at most N states; if more arrived, drop early ones (pre-trajectory)
    if len(actual_states) > N:
        actual_states = actual_states[-N:]
    print(f"[INFO] Captured {len(actual_states)}/{N} states via PUB socket")

    # Extract object trajectory from PUB states (100 Hz, covers whole
    # trajectory including the flight arc).
    # t is relative to trajectory start; release is at release_index * 0.01 s.
    obj_traj = []
    for i, s in enumerate(actual_states):
        gz = s.get('gazebo', {})
        if object_name in gz:
            traj_point = {
                't':   i * 0.01,
                'pos': gz[object_name]['position'],
            }
            if 'velocity' in gz[object_name]:
                traj_point['vel'] = gz[object_name]['velocity']
            obj_traj.append(traj_point)

    # Wait for landing via contact sensor
    print("[INFO] Waiting for landing detection...")
    landing = contact.wait_for_landing(timeout=3.0)
    if landing:
        print(f"[INFO] Landing detected: {landing['object_name']} at "
              f"[{landing['position'][0]:.3f}, {landing['position'][1]:.3f}, "
              f"{landing['position'][2]:.3f}]")
    else:
        print("[INFO] No landing detected (timeout)")

    # Short post-trajectory poll via REQ (now free) to capture settling
    t0_post = time.time()
    while time.time() - t0_post < 0.5:
        pose = gazebo.get_pose(object_name)
        if pose:
            traj_point = {
                't':   N * 0.01 + (time.time() - t0_post),
                'pos': pose['position'],
            }
            obj_traj.append(traj_point)
        time.sleep(0.02)

    print(f"[INFO] Object trajectory: {len(obj_traj)} samples "
          f"({sum(1 for o in obj_traj if o['t'] < N*0.01)} during traj, "
          f"{sum(1 for o in obj_traj if o['t'] >= N*0.01)} post-traj)")

    return {
        'planned_q':        Q,
        'planned_qd':       Qd,
        'commanded_ee_vel': commanded_ee_vel,
        'release_index':    release_index,
        'actual_states':    actual_states,
        'obj_traj':         obj_traj,
        'landing':          landing,
    }
