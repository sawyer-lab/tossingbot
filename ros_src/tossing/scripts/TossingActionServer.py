#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
from std_msgs.msg import String
from tossing.msg import TossAction, TossFeedback, TossResult
from intera_interface import Limb, Gripper
import time

from motion_planner import TossingPlanner

class TossingActionServer:
    def __init__(self):
        rospy.init_node('tossing_action_server')
        
        # hardware interfaces
        self.limb = Limb("right")
        self.gripper = Gripper()
        self.tip_name = "right_gripper_tip"
        
        # Logic helpers
        self.planner = TossingPlanner()
        self.control_pub = rospy.Publisher('recording_control', String, queue_size=1)
        
        # Action Server
        self.server = actionlib.SimpleActionServer('tossing_action', TossAction, self.execute_cb, False)
        self.server.start()
        
        rospy.loginfo("Tossing Action Server Ready.")

    def execute_cb(self, goal):
        result = TossResult()
        feedback = TossFeedback()
        

        feedback.current_step = "Planning"
        self.server.publish_feedback(feedback)
    
        base_angle_j0 = 0.0

        # 2. Plan Trajectory
        # try:
            # We want specific velocity at end effector
        speed = goal.speed
            
        start_time = time.time()
        sol = self.planner.get_trajectory(speed)
        end_time = time.time()
        rospy.loginfo("Planning time: {:.3f} seconds".format(end_time - start_time))
        full_traj = self.planner.map_to_7dof(
             sol['Q'], sol['Qd'], sol['Qdd'], base_angle_j0
        )
        # except Exception as e:
        #     rospy.logerr("Planning failed: {}".format(e))
        #     result.success = False
        #     self.server.set_aborted(result)
        #     return

        # 3. Execution
        # ------------
        feedback.current_step = "Executing"
        self.server.publish_feedback(feedback)
        
        success = self.execute_trajectory(full_traj)
        
        result.success = success
        if success:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def execute_trajectory(self, trajectory):
        q_full, qd_full, qdd_full = trajectory
        joint_names = self.limb.joint_names()
        rate = rospy.Rate(100)
        n = q_full.shape[0]

        self.send_control("start_vel")
        
        try:
            for i in range(n):
                if self.server.is_preempt_requested():
                    self.server.set_preempted()
                    return False
                
                # Send command
                self.limb.set_joint_trajectory(joint_names, q_full[i], qd_full[i], qdd_full[i])
                
                # Check for release condition (last point)
                if i == n - 1:
                    self.gripper.open()
                    
                rate.sleep()
        finally:
            self.send_control("stop_vel")
            self.send_control("start_pos")
            
        return True

    def send_control(self, cmd):
        self.control_pub.publish(String(data=cmd))

if __name__ == "__main__":
    TossingActionServer()
    rospy.spin()