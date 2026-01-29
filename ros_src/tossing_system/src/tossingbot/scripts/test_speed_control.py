#!/usr/bin/env python3.8
import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion

from tossingbot.hardware.sawyer import SawyerInterface, ControlMode
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot import config as cfg

# --- CONFIGURATION ---
ROBOT_Z_OFFSET = 1.0  # Robot base height in world frame

# Adjusted coordinates to stay safely on table
PICK_POS_WORLD = [0.65, -0.15, 0.775] 
PLACE_POS_WORLD = [0.65, 0.15, 0.775]

HOVER_OFFSET = 0.20
TARGET_QUAT = [0, 1, 0, 0] # Vertical grasp

def to_robot_frame(world_pos):
    """Converts world position [x,y,z] to robot frame."""
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p

def execute_trajectory(robot, plan_data):
    if plan_data is None: 
        rospy.logerr("Plan is None!")
        return False
    
    if isinstance(plan_data, list):
        stream = plan_data
    else:
        rospy.logerr("Unknown plan format")
        return False

    from tossingbot.hardware.sawyer import RobotCommand
    
    cmd_stream = [
        RobotCommand(
            position=p['position'],
            velocity=p['velocity'],
            acceleration=p['acceleration']
        ) for p in stream
    ]
    
    robot.execute_stream(cmd_stream, ControlMode.TRAJECTORY)
    return True

def run_routine(name, speeds, robot, gripper, planner):
    rospy.loginfo(f"--- STARTING ROUTINE: {name} ---")
    rospy.loginfo(f"Speeds: {speeds}")
    
    pick_robot = to_robot_frame(PICK_POS_WORLD)
    place_robot = to_robot_frame(PLACE_POS_WORLD)
    
    hover_pick = list(pick_robot)
    hover_pick[2] += HOVER_OFFSET
    
    hover_place = list(place_robot)
    hover_place[2] += HOVER_OFFSET

    # 1. Home
    rospy.loginfo(f"[{name}] Moving Home...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, duration=None, joint_speed=speeds['joint'])
    execute_trajectory(robot, traj)
    gripper.open()
    
    # 2. Hover Pick
    rospy.loginfo(f"[{name}] Hover Pick...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, hover_pick, TARGET_QUAT, duration=None, linear_speed=speeds['fast'])
    execute_trajectory(robot, traj)
    
    # 3. Approach
    rospy.loginfo(f"[{name}] Approach...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, pick_robot, TARGET_QUAT, duration=None, linear_speed=speeds['slow'])
    execute_trajectory(robot, traj)
    
    # 4. Grasp
    rospy.sleep(0.2) # Allow settling
    gripper.close()
    rospy.sleep(0.4)
    
    # 5. Lift
    rospy.loginfo(f"[{name}] Lift...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, hover_pick, TARGET_QUAT, duration=None, linear_speed=speeds['medium'])
    execute_trajectory(robot, traj)
    
    # 6. Transfer
    rospy.loginfo(f"[{name}] Transfer...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, hover_place, TARGET_QUAT, duration=None, linear_speed=speeds['fast'])
    execute_trajectory(robot, traj)
    
    # 7. Place Down
    rospy.loginfo(f"[{name}] Place...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, place_robot, TARGET_QUAT, duration=None, linear_speed=speeds['slow'])
    execute_trajectory(robot, traj)
    
    # 8. Release
    gripper.open()
    rospy.sleep(0.4)
    
    # 9. Retract
    rospy.loginfo(f"[{name}] Retract...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_cartesian(q_curr, hover_place, TARGET_QUAT, duration=None, linear_speed=speeds['fast'])
    execute_trajectory(robot, traj)
    
    rospy.loginfo(f"--- FINISHED ROUTINE: {name} ---")
    rospy.sleep(1.0)

def run_demo():
    rospy.init_node('speed_control_multidemo')
    
    # Setup
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    planner = CasadiPlanner(kinematics)
    
    # Define Speed Profiles
    # slow: precision
    # medium: normal op
    # fast: high speed (fast=transfer/hover, slow=approach/place)
    profiles = [
        {
            "name": "SLOW",
            "speeds": {"joint": 0.5, "fast": 0.2, "medium": 0.15, "slow": 0.05}
        },
        {
            "name": "MEDIUM",
            "speeds": {"joint": 1.0, "fast": 0.5, "medium": 0.25, "slow": 0.1}
        },
        {
            "name": "FAST",
            "speeds": {"joint": 1.5, "fast": 0.8, "medium": 0.4, "slow": 0.15}
        }
    ]
    
    for prof in profiles:
        # Reset Scene
        rospy.loginfo("Resetting Scene...")
        manager.despawn("demo_cube")
        rospy.sleep(0.5)
        manager.spawn("cube", "demo_cube", Pose(
            position=Point(*PICK_POS_WORLD), 
            orientation=Quaternion(0,0,0,1)
        ))
        rospy.sleep(1.0)
        
        # Run
        run_routine(prof['name'], prof['speeds'], robot, gripper, planner)

    rospy.loginfo("All Demos Complete!")

if __name__ == "__main__":
    try:
        run_demo()
    except rospy.ROSInterruptException:
        pass