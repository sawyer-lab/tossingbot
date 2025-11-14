#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from simulation.msg import TossAction, TossFeedback, TossResult
from intera_interface import Limb, Gripper
import numpy as np
from motion_planner import solve_trajectory_problem, scale_solution
from std_msgs.msg import String

l1 = 0.4
l2 = 0.4
l3 = 0.13375
# gripper = 0.13562
gripper = 0.1944

base_offset_x = 0.081
base_offset_y = 0.0
base_offset_z = 0.317

global_gripper_width = 0.01
global_gripper_length = 0.01


def inverse_kinematics_3r(pose):
    x, z, t3 = pose
    # subtract base offsets
    x_rel = x - base_offset_x
    z_rel = z - base_offset_z
    
    # wrist position
    x_wrist = x_rel - (l3 + gripper) * np.cos(t3)
    z_wrist = z_rel - (l3 + gripper) * np.sin(t3)

    # compute D
    D = (x_wrist**2 + z_wrist**2 - l1**2 - l2**2) / (2*l1*l2)
    if abs(D) > 1:
        raise ValueError("Target is out of reach")

    q2 = np.arctan2(-np.sqrt(1 - D**2), D)
    q1 = np.arctan2(z_wrist, x_wrist) - np.arctan2(l2*np.sin(q2), l1+l2*np.cos(q2))
    q3 = t3 - q1 - q2

    # match FK sign convention
    return np.array([-q1, -q2, -q3])

class TossingActionServer:
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer('tossing_action', TossAction, self.execute, False)
        self.server.start()
        self.hover_distance = 0.15
        self.tip_name = "right_gripper_tip"
        self.rate = rospy.Rate(100)
        self.relase_time = 0.7
        rospy.loginfo("Tossing Action Server is ready.")
        
    def execute(self, goal):
                
        feedback = TossFeedback()
        result = TossResult()
        
        try:
            self.limb = Limb("right")
            self.gripper = Gripper()
        except:
            rospy.logerr("Failed to initialize limb or gripper interface.")
            result.success = False
            self.server.set_aborted(result)
            return
        
        
        
        feedback.current_step = "Planning"
        self.server.publish_feedback(feedback)
        # sol = solve_trajectory_problem(
        #     np.array([goal.target_velocity.linear.x, goal.target_velocity.linear.y, 0.0]),
        #     self.relase_time
        # )
        

        T = 0.7
        q = np.array([-1.15, 1.7, 1.5])
        # q = inverse_kinematics_3r(np.array([0.175, 0.025, -2.094395102393195]))
        # xT = np.array([0.825, 0.0, 0.0])
        xT = np.array([0.825, 0.3, 0.])
        angle = 45.0 * (np.pi / 180.0)  # radians
        speed = 2.0
        v_final = np.array([speed * np.cos(angle), speed * np.sin(angle), 0.0])

    # 3. Solve the optimization problem
        sol = solve_trajectory_problem(v_final, T, q, xT)

        scaled_sol = scale_solution(sol, 1.5)
        sol = scaled_sol

        
        # sol = solve_trajectory_problem_scaled(v_final, q, xT, scale=0.75)
        
        theta = np.arctan2(goal.target_position.y, goal.target_position.x)
        
        trajectory = TossingActionServer.assemble_full_trajectory(sol['Q'], sol['Qd'], sol['Qdd'], theta)


        
        feedback.current_step = "Executing"
        self.server.publish_feedback(feedback)
        self.stream_trajectory(trajectory, 0)
        
        result.success = True
        self.server.set_succeeded(result)
        
    @staticmethod
    def assemble_full_trajectory(q, qd, qdd, q0=0.0):

        fixed_values = {
                "j0": q0,
                "j2": 0.0,
                "j4": 0.0,
                "j6": 0.0,
                # "j6": 1.7659902159840577,
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
        qdd_full[:, 1] = qdd[0] - np.pi/2
        qdd_full[:, 3] = qdd[1]
        qdd_full[:, 5] = qdd[2]

        for idx, joint in zip([0, 2, 4, 6], ["j0", "j2", "j4", "j6"]):
            val = fixed_values[joint]
            q_full[:, idx] = val
            qd_full[:, idx] = 0.0
            qdd_full[:, idx] = 0.0

        return q_full, qd_full, qdd_full

    def stream_trajectory(self, trajectory, release_index):

        q_full, qd_full, qdd_full = trajectory 
        joint_names = self.limb.joint_names()
        

        send_control_signal("start_vel")
        n = q_full.shape[0]
        for i, (q, dq, ddq) in enumerate(zip(q_full, qd_full, qdd_full)):
            self.limb.set_joint_trajectory(joint_names, q, dq, ddq)
            if i == n -1 :
                vel = self.limb.endpoint_velocity()
                rospy.loginfo("Releasing at velocity: x={:.3f}, y={:.3f}, z={:.3f}".format(vel['linear'][0], vel['linear'][1], vel['linear'][2]))
                magnitude = np.linalg.norm([vel['linear'][0], vel['linear'][1], vel['linear'][2]])
                rospy.loginfo("Velocity magnitude: {:.3f} m/s".format(magnitude))
                self.gripper.open()
            self.rate.sleep()
        send_control_signal("stop_vel")
        send_control_signal("start_pos")
            

def send_control_signal(command):
    pub = rospy.Publisher('recording_control', String, queue_size=1)
    msg = String(data=command)
    pub.publish(msg)
    # rospy.loginfo("Sent command: %s", command)
           

if __name__ == "__main__":
    rospy.init_node('tossing_action_server')
    server = TossingActionServer()
    rospy.spin()