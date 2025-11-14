#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from intera_interface import Limb
import csv
import os
from datetime import datetime
from gazebo_msgs.srv import GetModelState

class MonitorNode(object):
    def __init__(self):
        self.recording_vel = False
        self.recording_pos = False
        self.limb = Limb("right")
        self.timestamps = []
        self.velocities = {'x': [], 'y': [], 'z': []}
        self.positions = {'x': [], 'y': [], 'z': []}
        self.csv_file_vel = None
        self.csv_writer_vel = None
        self.csv_file_pos = None
        self.csv_writer_pos = None

        # Wait for the GetModelState service
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.Subscriber('recording_control', String, self.control_callback)
        rospy.loginfo("MonitorNode ready, waiting for 'start_vel'/'stop_vel' or 'start_pos'/'stop_pos' on /recording_control...")

    def control_callback(self, msg):
        if msg.data == "start_vel":
            if not self.recording_vel:
                self.start_recording_vel()
        elif msg.data == "stop_vel":
            if self.recording_vel:
                self.stop_recording_vel()
        elif msg.data == "start_pos":
            if not self.recording_pos:
                self.start_recording_pos()
        elif msg.data == "stop_pos":
            if self.recording_pos:
                self.stop_recording_pos()

    def start_recording_vel(self):
        self.recording_vel = True
        self.timestamps = []
        self.velocities = {'x': [], 'y': [], 'z': []}

        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data"))
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        file_name = "endpoint_velocity_{}.csv".format(now)
        self.csv_file_vel = open(os.path.join(save_dir, file_name), mode='w')
        self.csv_writer_vel = csv.writer(self.csv_file_vel)
        self.csv_writer_vel.writerow(["Time (s)", "Linear Velocity X", "Linear Velocity Y", "Linear Velocity Z"])
        rospy.loginfo("Started recording endpoint velocity!")

    def stop_recording_vel(self):
        self.recording_vel = False
        if self.csv_file_vel:
            self.csv_file_vel.close()
            self.csv_file_vel = None
            self.csv_writer_vel = None
        rospy.loginfo("Stopped recording endpoint velocity! Data saved.")

    def start_recording_pos(self):
        self.recording_pos = True
        self.positions = {'x': [], 'y': [], 'z': []}

        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data"))
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        file_name = "block_position_{}.csv".format(now)
        self.csv_file_pos = open(os.path.join(save_dir, file_name), mode='w')
        self.csv_writer_pos = csv.writer(self.csv_file_pos)
        self.csv_writer_pos.writerow(["Time (s)", "Position X", "Position Y", "Position Z"])
        rospy.loginfo("Started recording block position!")

    def stop_recording_pos(self):
        self.recording_pos = False
        if self.csv_file_pos:
            self.csv_file_pos.close()
            self.csv_file_pos = None
            self.csv_writer_pos = None
        rospy.loginfo("Stopped recording block position! Data saved.")

    def record_velocity(self):
        vel = self.limb.endpoint_velocity()
        t = rospy.get_time()
        self.timestamps.append(t)
        self.velocities['x'].append(vel['linear'].x)
        self.velocities['y'].append(vel['linear'].y)
        self.velocities['z'].append(vel['linear'].z)

        if self.csv_writer_vel:
            self.csv_writer_vel.writerow([t, vel['linear'].x, vel['linear'].y, vel['linear'].z])

    def record_position(self):
        try:
            resp = self.get_model_state('block', 'world')
            t = rospy.get_time()
            x = resp.pose.position.x
            y = resp.pose.position.y
            z = resp.pose.position.z
            vel = resp.twist.linear
            self.positions['x'].append(x)
            self.positions['y'].append(y)
            self.positions['z'].append(z)
            if self.csv_writer_pos:
                self.csv_writer_pos.writerow([t, x, y, z, vel.x, vel.y, vel.z])
        except rospy.ServiceException as e:
            rospy.logwarn("GetModelState service call failed: {}".format(e))

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.recording_vel:
                self.record_velocity()
            if self.recording_pos:
                self.record_position()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('monitor_node')
    node = MonitorNode()
    node.spin()