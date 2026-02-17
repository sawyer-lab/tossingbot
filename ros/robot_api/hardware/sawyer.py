#!/usr/bin/env python3.8
import rospy
import numpy as np
from intera_core_msgs.msg import JointCommand, EndpointState
from sensor_msgs.msg import JointState
from typing import List, Dict, Optional
from dataclasses import dataclass, field

@dataclass
class RobotCommand:
    """
    Holds the command for a SINGLE 10ms timestep.
    Populate only the fields relevant to your mode.
    """
    position: List[float] = field(default_factory=list)
    velocity: List[float] = field(default_factory=list)
    acceleration: List[float] = field(default_factory=list) 
    effort: List[float] = field(default_factory=list)       

class ControlMode:
    POSITION = JointCommand.POSITION_MODE
    VELOCITY = JointCommand.VELOCITY_MODE
    TORQUE = JointCommand.TORQUE_MODE
    TRAJECTORY = JointCommand.TRAJECTORY_MODE


class SawyerInterface:
    def __init__(self):
        self._ns = '/robot/limb/right/'
        self._joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 
                             'right_j4', 'right_j5', 'right_j6']
        self._control_rate = 100.0
        self._rate = rospy.Rate(self._control_rate)

        # State Management
        self._curr_joints = {}
        self._curr_joint_velocities = {}
        self._curr_joint_efforts = {}
        self._received_state = False
        self._endpoint_pose = None
        self._endpoint_velocity = None
        self._endpoint_effort = None
        self._received_endpoint = False

        # Publisher (High Priority TCP)
        self._pub_joint_cmd = rospy.Publisher(
            self._ns + 'joint_command',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1
        )
        
        # Pre-allocate message
        self._command_msg = JointCommand()
        self._command_msg.names = self._joint_names

        # Subscriber (Joint States)
        rospy.Subscriber(
            '/robot/joint_states', 
            JointState, 
            self._cb_joint_states,
            queue_size=1,
            tcp_nodelay=True
        )
        
        # Subscriber (Endpoint State)
        rospy.Subscriber(
            self._ns + 'endpoint_state',
            EndpointState,
            self._cb_endpoint_state,
            queue_size=1,
            tcp_nodelay=True
        )

        # Block until Hardware is Ready
        rospy.loginfo("SawyerInterface: Waiting for robot state...")
        while not self._received_state and not rospy.is_shutdown():
            self._rate.sleep()
        rospy.loginfo("SawyerInterface: Online.")

    def _cb_joint_states(self, msg: JointState):
        if 'right_j0' in msg.name:
            temp_pos = dict(zip(msg.name, msg.position))
            temp_vel = dict(zip(msg.name, msg.velocity))
            temp_eff = dict(zip(msg.name, msg.effort))
            self._curr_joints = {n: temp_pos[n] for n in self._joint_names}
            self._curr_joint_velocities = {n: temp_vel[n] for n in self._joint_names}
            self._curr_joint_efforts = {n: temp_eff[n] for n in self._joint_names}
            self._received_state = True
    
    def _cb_endpoint_state(self, msg: EndpointState):
        """Callback for endpoint state - stores current pose, velocity, and effort"""
        self._endpoint_pose = {
            'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'orientation': [msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w]
        }
        self._endpoint_velocity = {
            'linear': [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            'angular': [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        }
        self._endpoint_effort = {
            'force': [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            'torque': [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        }
        self._received_endpoint = True

    def get_endpoint_pose(self) -> Optional[Dict]:
        """
        Returns the current end-effector pose.
        Returns:
            dict with 'position': [x, y, z], 'orientation': [x, y, z, w]
            or None if not yet received
        """
        return self._endpoint_pose if self._received_endpoint else None

    def get_endpoint_velocity(self) -> Optional[Dict]:
        """
        Returns the current end-effector velocity (twist).
        Returns:
            dict with 'linear': [vx, vy, vz], 'angular': [wx, wy, wz] in m/s and rad/s
            or None if not yet received
        """
        return self._endpoint_velocity if self._received_endpoint else None

    def get_endpoint_effort(self) -> Optional[Dict]:
        """
        Returns the current end-effector effort (wrench).
        Returns:
            dict with 'force': [fx, fy, fz], 'torque': [tx, ty, tz] in N and Nm
            or None if not yet received
        """
        return self._endpoint_effort if self._received_endpoint else None

    def get_joint_positions(self) -> List[float]:
        """Returns [q0, q1, ... q6] in the correct order."""
        if not self._received_state: return [0.0] * 7
        return [self._curr_joints[n] for n in self._joint_names]

    def get_joint_velocities(self) -> List[float]:
        """Returns joint velocities [qd0, qd1, ... qd6] in rad/s."""
        if not self._received_state: return [0.0] * 7
        return [self._curr_joint_velocities[n] for n in self._joint_names]

    def get_joint_efforts(self) -> List[float]:
        """Returns joint efforts [tau0, tau1, ... tau6] in Nm."""
        if not self._received_state: return [0.0] * 7
        return [self._curr_joint_efforts[n] for n in self._joint_names]
    
    def move_to_joint_positions(self, target_joints: List[float], timeout: float = 1.0):
        """
        Moves to target joint positions with a strict TIMEOUT.
        Prevents infinite loops if the robot cannot reach the exact tolerance.
        """
        tolerance = 0.015 
        max_error = 100.0
        
        cmd = RobotCommand(position=target_joints)
        chunk = [cmd] * 10 
        
        start_time = rospy.Time.now()

        while max_error > tolerance and not rospy.is_shutdown():
            # --- 1. TIMEOUT CHECK ---
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn(f"MOVE TIMEOUT: Aborting. Final Error: {max_error:.4f} rad")
                break 

            self.execute_stream(chunk, ControlMode.POSITION)
            current = self.get_joint_positions()
            errors = [abs(c - t) for c, t in zip(current, target_joints)]
            max_error = max(errors)


    def execute_stream(self, stream: List[RobotCommand], mode: int) -> bool:
        """
        BLOCKING CALL.
        Streams a sequence of commands at exactly 100Hz.
        
        Args:
            stream: A list of RobotCommand objects (the trajectory).
            mode:   ControlMode (POSITION, VELOCITY, TRAJECTORY, TORQUE).
        """
        if not stream:
            rospy.logwarn("SawyerInterface: Received empty stream.")
            return False

        self._command_msg.mode = mode

        for step in stream:
            if rospy.is_shutdown(): return False

            # 1. Update Message
            if mode == ControlMode.POSITION:
                self._command_msg.position = step.position
            elif mode == ControlMode.VELOCITY:
                self._command_msg.velocity = step.velocity
            elif mode == ControlMode.TORQUE:
                self._command_msg.effort = step.effort
            elif mode == ControlMode.TRAJECTORY:
                self._command_msg.position = step.position
                self._command_msg.velocity = step.velocity
                self._command_msg.acceleration = step.acceleration
            
            # 2. Stamp & Send
            self._command_msg.header.stamp = rospy.Time.now()
            self._pub_joint_cmd.publish(self._command_msg)
            
            # 3. Wait (Enforce 100Hz)
            self._rate.sleep()

        return True
    


if __name__ == "__main__":
    """Simple Test Script for SawyerInterface
    Moves the robot to a target joint configuration using a simple feedback loop.
    """
    rospy.init_node("sawyer_interface_test")
    
    # 1. Init Driver
    robot = SawyerInterface()
    
    # 2. Get Current Position
    start_joints = robot.get_joint_positions()
    print(f"Current Joints: {np.round(start_joints, 3)}")

    # 3. Define a Target (Move Joint 0 by 0.5 rad)
    # Be careful not to hit yourself!
    target_joints = list(start_joints)
    target_joints[0] = 0.0 
    target_joints[1] = 0.3
    target_joints[2] = 0.5
    target_joints[3] = 0.0
    target_joints[4] = 0.0
    target_joints[5] = -0.2
    target_joints[6] = 0.0
    
    print(f"Target Joints:  {np.round(target_joints, 3)}")
    print("Moving in 1 second...")
    rospy.sleep(1.0)

    # 4. The "Simple Planner" Loop
    # Mimics SDK: Sends command repeatedly until error is low
    tolerance = 0.02
    max_error = 100.0
    
    # We create a short stream (chunk) to send in each loop iteration
    # This keeps the heartbeat happy (100Hz) while allowing us to check error every 0.1s
    cmd = RobotCommand(position=target_joints)
    chunk = [cmd] * 10 # 0.1 seconds worth of commands

    while max_error > tolerance and not rospy.is_shutdown():
        # A. Execute short stream (Keep robot alive)
        robot.execute_stream(chunk, ControlMode.POSITION)
        
        # B. Check Error (Feedback)
        current = robot.get_joint_positions()
        errors = [abs(c - t) for c, t in zip(current, target_joints)]
        max_error = max(errors)
        
        print(f"\rMax Error: {max_error:.4f} rad", end="")

    print("\n\nTarget Reached!")
    
    # 5. Hold Position (Prevent gravity sag)
    print("Holding position for 2 seconds...")
    hold_chunk = [RobotCommand(position=target_joints)] * 200 # 2.0 seconds
    robot.execute_stream(hold_chunk, ControlMode.POSITION)
    
    print("Done.")