#!/usr/bin/env python

import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from intera_interface import Limb
from copy import deepcopy
import signal
import time
import os
from datetime import datetime

running = True

# Endpoint data
timestamps = []
positions = {'x': [], 'y': [], 'z': []}
linear_vels = {'x': [], 'y': [], 'z': []}
linear_accs = {'x': [], 'y': [], 'z': []}

# Joint data
joint_names = []
joint_positions = {}

def signal_handler(sig, frame):
    global running
    print("\nCtrl+C received. Stopping data collection and plotting...")
    running = False

def is_all_zero(pose, vel):
    return (
        abs(vel['linear'].x) < 1e-6 and
        abs(vel['linear'].y) < 1e-6 and
        abs(vel['linear'].z) < 1e-6
    )

def compute_acceleration(curr, prev, dt):
    if dt <= 0.0:
        return 0.0
    return (curr - prev) / dt

def plot_data():
    fig, axs = plt.subplots(3, 3, figsize=(15, 8))
    fig.suptitle("Endpoint Position, Velocity, and Acceleration Over Time")

    axes = ['x', 'y', 'z']
    titles = ['Position', 'Linear Velocity', 'Linear Acceleration']
    data = [positions, linear_vels, linear_accs]

    for row in range(3):
        for col in range(3):
            axs[row, col].plot(timestamps, data[row][axes[col]], label=axes[col])
            axs[row, col].set_title("{} ({})".format(titles[row], axes[col]))
            axs[row, col].legend()

    plt.tight_layout()
    plt.subplots_adjust(top=0.9)

    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../images"))
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    save_path = os.path.join(save_dir, "endpoint_plot_{}.png".format(now))
    plt.savefig(save_path)
    print("Endpoint plot saved to: {}".format(save_path))

def plot_joint_positions():
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.suptitle("Joint Positions Over Time")

    for name in joint_names:
        ax.plot(timestamps, joint_positions[name], label=name)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Joint Angle (rad)")
    ax.legend()
    plt.tight_layout()

    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../images"))
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    save_path = os.path.join(save_dir, "joint_positions_{}.png".format(now))
    plt.savefig(save_path)
    print("Joint position plot saved to: {}".format(save_path))

def main():
    global running

    rospy.init_node('pose_velocity_logger')
    signal.signal(signal.SIGINT, signal_handler)

    limb = Limb("right")
    rate = rospy.Rate(100)  # 10 Hz
    start_time = time.time()

    # Initialize joint data
    global joint_names
    joint_names = limb.joint_names()
    for name in joint_names:
        joint_positions[name] = []

    prev_vel = None
    prev_time = None

    print("Recording endpoint and joint data. Press Ctrl+C to stop and plot.")

    while not rospy.is_shutdown() and running:
        pose = deepcopy(limb.endpoint_pose())
        vel = deepcopy(limb.endpoint_velocity())

        if is_all_zero(pose, vel):
            rate.sleep()
            continue

        t = time.time() - start_time
        timestamps.append(t)

        # Endpoint Positions
        px, py, pz = pose['position'].x, pose['position'].y, pose['position'].z
        positions['x'].append(px)
        positions['y'].append(py)
        positions['z'].append(pz)

        # Linear velocities
        vx, vy, vz = vel['linear'].x, vel['linear'].y, vel['linear'].z
        linear_vels['x'].append(vx)
        linear_vels['y'].append(vy)
        linear_vels['z'].append(vz)

        # Linear accelerations
        if prev_vel is not None:
            dt = t - prev_time
            linear_accs['x'].append(compute_acceleration(vx, prev_vel[0], dt))
            linear_accs['y'].append(compute_acceleration(vy, prev_vel[1], dt))
            linear_accs['z'].append(compute_acceleration(vz, prev_vel[2], dt))
        else:
            linear_accs['x'].append(0.0)
            linear_accs['y'].append(0.0)
            linear_accs['z'].append(0.0)

        prev_vel = (vx, vy, vz)
        prev_time = t

        # Record joint angles using ordered list
        angles = limb.joint_ordered_angles()
        for i, name in enumerate(joint_names):
            joint_positions[name].append(angles[i])

        rate.sleep()

    # Plot both endpoint data and joint positions
    plot_data()
    plot_joint_positions()

if __name__ == '__main__':
    main()
