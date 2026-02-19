#!/usr/bin/env python3
"""
ZeroMQ Server for Robot Control

Provides two communication patterns:
  - REP socket (port 5555): Command/response for robot control
  - PUB socket (port 5556): State broadcasting at 100Hz

Run this INSIDE the container.
"""

import zmq
import rospy
import json
import threading
import time
import sys
import os
import base64

# Add robot_api to path
sys.path.insert(0, '/robot_api')

from hardware.robot import Robot
from hardware.gripper import Gripper
from hardware.camera import Camera, CAMERA_TOPIC_MAP
from hardware.lights import Lights
from hardware.head import Head
from hardware.head_display import HeadDisplay
from hardware.gazebo_manager import GazeboManager
from hardware.contact_sensor_manager import ContactSensorManager
from hardware.rgbd_camera_manager import RGBDCameraManager


class RobotServer:
    """ZeroMQ server for robot and gripper control"""

    def __init__(self, command_port=5555, state_port=5556):
        """
        Initialize the ZMQ server.

        Args:
            command_port: Port for command socket (REP)
            state_port: Port for state broadcast (PUB)
        """
        self.command_port = command_port
        self.state_port = state_port
        self.running = True

        # Initialize ROS node
        rospy.init_node('robot_zmq_server', anonymous=True)

        # Initialize robot hardware
        rospy.loginfo("Initializing Robot...")
        self.robot = Robot()

        rospy.loginfo("Initializing Gripper...")
        self.gripper = Gripper()

        rospy.loginfo("Initializing Cameras...")
        self.cameras = {
            'head': Camera('head_camera'),
            'hand': Camera('right_hand_camera'),
        }

        rospy.loginfo("Initializing Lights...")
        self.lights = Lights()

        rospy.loginfo("Initializing Head...")
        self.head = Head()

        rospy.loginfo("Initializing Head Display...")
        self.head_display = HeadDisplay()

        rospy.loginfo("Initializing Gazebo Manager...")
        try:
            self.gazebo = GazeboManager()
        except Exception as e:
            rospy.logwarn(f"Gazebo Manager not available: {e}")
            self.gazebo = None

        rospy.loginfo("Initializing Contact Sensor...")
        try:
            self.contact_sensor = ContactSensorManager()
        except Exception as e:
            rospy.logwarn(f"Contact Sensor not available: {e}")
            self.contact_sensor = None

        rospy.loginfo("Initializing RGBD Camera...")
        try:
            self.rgbd_camera = RGBDCameraManager()
            self.camera_streaming_enabled = False
        except Exception as e:
            rospy.logwarn(f"RGBD Camera not available: {e}")
            self.rgbd_camera = None
            self.camera_streaming_enabled = False

        # Create ZeroMQ context and sockets
        self.context = zmq.Context()

        # REP socket for commands (request/response pattern)
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.bind(f"tcp://*:{self.command_port}")

        # PUB socket for state broadcasting (publish/subscribe pattern)
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.state_port}")

        rospy.loginfo(f"ZMQ Server initialized on ports {command_port}, {state_port}")

    def get_full_state(self):
        """Get complete robot, gripper, Gazebo, and sensor state."""
        robot_state   = self.robot.get_state()
        gripper_state = self.gripper.get_state()

        # Serialize Gazebo pose cache (updated at 100Hz via /gazebo/model_states).
        # Using the cache directly avoids any ROS service call overhead.
        gazebo_state = {}
        if self.gazebo is not None:
            with self.gazebo._lock:
                for name in self.gazebo._pose_cache.keys():
                    pose = self.gazebo._pose_cache.get(name)
                    twist = self.gazebo._twist_cache.get(name)
                    
                    if pose:
                        gazebo_state[name] = {
                            'position': [pose.position.x,
                                         pose.position.y,
                                         pose.position.z],
                            'orientation': [pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w],
                        }
                        
                        if twist:
                            gazebo_state[name]['velocity'] = [twist.linear.x,
                                                               twist.linear.y,
                                                               twist.linear.z]
                            gazebo_state[name]['angular_velocity'] = [twist.angular.x,
                                                                       twist.angular.y,
                                                                       twist.angular.z]

        # Add contact sensor data
        contact_state = None
        if self.contact_sensor is not None:
            landing = self.contact_sensor.get_last_landing()
            if landing:
                contact_state = landing

        # Add camera data if streaming is enabled
        camera_state = None
        if self.camera_streaming_enabled and self.rgbd_camera is not None:
            cloud = self.rgbd_camera.get_point_cloud()
            if cloud:
                # Convert numpy arrays to lists for JSON serialization
                camera_state = {
                    'points': cloud['points'].tolist(),
                    'colors': cloud['colors'].tolist(),
                    'timestamp': cloud['timestamp']
                }

        return {
            'robot':     robot_state,
            'gripper':   gripper_state,
            'gazebo':    gazebo_state,
            'contact':   contact_state,
            'camera':    camera_state,
            'timestamp': time.time(),
        }

    def state_publisher_thread(self):
        """Thread that continuously publishes robot state at 100Hz"""
        rospy.loginfo("State publisher thread started (broadcasting on port {})".format(self.state_port))

        rate = rospy.Rate(100)  # 100Hz

        while self.running and not rospy.is_shutdown():
            try:
                state = self.get_full_state()
                self.pub_socket.send_json(state)
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"State publisher error: {e}")
                time.sleep(0.1)

        rospy.loginfo("State publisher thread stopped")

    def _get_camera(self, data):
        """Return the Camera instance for the requested camera name (default: head)."""
        name = data.get('camera', 'head')
        if name not in self.cameras:
            rospy.logwarn(f"Unknown camera '{name}', falling back to 'head'")
            name = 'head'
        return self.cameras[name]

    def handle_command(self, message):
        """
        Process incoming command and return response.

        Args:
            message: Dict with 'command' and optional arguments

        Returns:
            Dict with 'status' and result data
        """
        command = message.get('command')

        try:
            # ===== Robot Commands =====
            if command == 'move_robot':
                angles = message.get('angles')
                timeout = message.get('timeout', 30.0)

                if not angles:
                    return {'status': 'error', 'message': 'Missing angles'}
                if len(angles) != 7:
                    return {'status': 'error', 'message': 'Expected 7 angles'}

                success = self.robot.move_to_joints(angles, timeout=timeout)

                if success:
                    return {'status': 'ok', 'angles': angles}
                else:
                    return {'status': 'error', 'message': 'Movement timed out or failed'}

            elif command == 'get_angles':
                angles = self.robot.get_joint_angles()
                return {'status': 'ok', 'angles': angles}

            elif command == 'get_endpoint_pose':
                pose = self.robot.get_endpoint_pose()
                return {'status': 'ok', 'pose': pose}

            elif command == 'get_robot_state':
                state = self.robot.get_state()
                return {'status': 'ok', 'state': state}

            elif command == 'get_joint_velocities':
                velocities = self.robot.get_joint_velocities()
                return {'status': 'ok', 'velocities': velocities}

            elif command == 'get_endpoint_velocity':
                velocity = self.robot.get_endpoint_velocity()
                if velocity is not None:
                    return {'status': 'ok', 'velocity': velocity}
                return {'status': 'error', 'message': 'No velocity data available'}

            elif command == 'execute_trajectory':
                waypoints = message.get('waypoints')
                rate_hz = message.get('rate_hz', 100.0)
                if not waypoints:
                    return {'status': 'error', 'message': 'Missing waypoints'}
                success = self.robot.execute_trajectory(waypoints, rate_hz)
                return {'status': 'ok' if success else 'error'}

            elif command == 'execute_toss_trajectory':
                # Execute trajectory at 100Hz with async gripper release at release_index.
                # The gripper fires a non-blocking ROS publish in a daemon thread so the
                # trajectory loop is never interrupted.
                import numpy as np
                Q   = np.array(message['Q'],   dtype=float)   # (N, 7)
                Qd  = np.array(message['Qd'],  dtype=float)   # (N, 7)
                Qdd = np.array(message['Qdd'], dtype=float)   # (N, 7)
                release_index = message.get('release_index', None)

                from hardware.sawyer import ControlMode
                sawyer = self.robot._sawyer
                sawyer._command_msg.mode = ControlMode.TRAJECTORY
                rate = rospy.Rate(100)

                def _fire_gripper():
                    self.gripper.release()

                N = len(Q)
                for k in range(N):
                    if rospy.is_shutdown():
                        return {'status': 'error', 'message': 'ROS shutdown'}

                    if release_index is not None and k == release_index:
                        t = threading.Thread(target=_fire_gripper, daemon=True)
                        t.start()

                    sawyer._command_msg.position    = Q[k].tolist()
                    sawyer._command_msg.velocity    = Qd[k].tolist()
                    sawyer._command_msg.acceleration = Qdd[k].tolist()
                    sawyer._command_msg.header.stamp = rospy.Time.now()
                    sawyer._pub_joint_cmd.publish(sawyer._command_msg)
                    rate.sleep()

                return {'status': 'ok'}

            # ===== Gripper Commands =====
            elif command == 'gripper_open':
                success = self.gripper.open()
                return {'status': 'ok' if success else 'error'}

            elif command == 'gripper_close':
                success = self.gripper.close()
                return {'status': 'ok' if success else 'error'}

            elif command == 'gripper_set_position':
                position = message.get('position')
                if position is None:
                    return {'status': 'error', 'message': 'Missing position'}

                success = self.gripper.set_position(position)
                return {'status': 'ok' if success else 'error'}

            elif command == 'gripper_get_state':
                state = self.gripper.get_state()
                return {'status': 'ok', 'state': state}

            # ===== Camera Commands =====
            elif command == 'camera_start':
                cam = self._get_camera(message)
                success = cam.start_streaming()
                return {'status': 'ok' if success else 'error'}

            elif command == 'camera_stop':
                cam = self._get_camera(message)
                success = cam.stop_streaming()
                return {'status': 'ok' if success else 'error'}

            elif command == 'camera_get_image':
                cam = self._get_camera(message)
                image_bytes = cam.get_image_compressed()
                if image_bytes:
                    return {'status': 'ok', 'image': image_bytes.hex()}
                return {'status': 'error', 'message': 'No image available'}

            elif command == 'camera_get_state':
                cam = self._get_camera(message)
                return {'status': 'ok', 'state': cam.get_state()}

            # ===== Lights Commands =====
            elif command == 'lights_list':
                lights = self.lights.list_all_lights()
                return {'status': 'ok', 'lights': lights}

            elif command == 'lights_set':
                name = message.get('name')
                on = message.get('on', True)
                if not name:
                    return {'status': 'error', 'message': 'Missing name'}
                success = self.lights.set_light_state(name, on)
                return {'status': 'ok' if success else 'error'}

            elif command == 'lights_get':
                name = message.get('name')
                if not name:
                    return {'status': 'error', 'message': 'Missing name'}
                state = self.lights.get_light_state(name)
                return {'status': 'ok', 'on': state}

            # ===== Head Commands =====
            elif command == 'head_pan':
                return {'status': 'ok', 'angle': self.head.pan()}

            elif command == 'head_set_pan':
                angle = message.get('angle')
                speed = message.get('speed', 1.0)
                if angle is None:
                    return {'status': 'error', 'message': 'Missing angle'}
                success = self.head.set_pan(angle, speed)
                return {'status': 'ok' if success else 'error'}

            # ===== Head Display Commands =====
            elif command == 'display_image':
                img_hex = message.get('image')
                if not img_hex:
                    return {'status': 'error', 'message': 'Missing image'}
                import cv2
                import numpy as np
                img_bytes = bytes.fromhex(img_hex)
                nparr = np.frombuffer(img_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                success = self.head_display.display_image(cv_image=img)
                return {'status': 'ok' if success else 'error'}

            elif command == 'display_clear':
                success = self.head_display.clear()
                return {'status': 'ok' if success else 'error'}

            # ===== Robot Enable Commands =====
            elif command == 'robot_enable':
                success = self.robot_enable.enable()
                return {'status': 'ok' if success else 'error'}

            elif command == 'robot_disable':
                success = self.robot_enable.disable()
                return {'status': 'ok' if success else 'error'}

            elif command == 'robot_reset':
                success = self.robot_enable.reset()
                return {'status': 'ok' if success else 'error'}

            elif command == 'robot_stop':
                success = self.robot_enable.stop()
                return {'status': 'ok' if success else 'error'}

            elif command == 'robot_state':
                state_msg = self.robot_enable.state()
                if state_msg:
                    state = {
                        'enabled': state_msg.enabled,
                        'stopped': state_msg.stopped,
                        'error': state_msg.error,
                        'estop_button': state_msg.estop_button,
                        'estop_source': state_msg.estop_source,
                    }
                    return {'status': 'ok', 'state': state}
                return {'status': 'error', 'message': 'No state available'}

            # ===== Robot Params Commands =====
            elif command == 'params_robot_name':
                name = self.robot_params.get_robot_name()
                return {'status': 'ok', 'name': name}

            elif command == 'params_limb_names':
                limbs = self.robot_params.get_limb_names()
                return {'status': 'ok', 'limbs': limbs}

            elif command == 'params_joint_names':
                limb = message.get('limb', 'right')
                joints = self.robot_params.get_joint_names(limb)
                return {'status': 'ok', 'joints': joints}

            elif command == 'params_camera_names':
                cameras = self.robot_params.get_camera_names()
                return {'status': 'ok', 'cameras': cameras}

            elif command == 'params_all':
                info = self.robot_params.get_all_info()
                return {'status': 'ok', 'info': info}

            # ===== Gazebo Commands =====
            elif command == 'gazebo_spawn_sdf':
                # Low-level: spawn from SDF XML (host provides SDF)
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                name = message.get('name')
                sdf_xml = message.get('sdf_xml')
                position = message.get('position')
                orientation = message.get('orientation', [0, 0, 0, 1])
                if not name or not sdf_xml or not position:
                    return {'status': 'error', 'message': 'Missing required parameters'}

                # Create Pose from position/orientation
                from geometry_msgs.msg import Pose, Point, Quaternion
                pose = Pose()
                pose.position = Point(position[0], position[1], position[2])
                pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

                success = self.gazebo.spawn_sdf(name, sdf_xml, pose)
                return {'status': 'ok' if success else 'error'}

            elif command == 'gazebo_delete':
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                name = message.get('name')
                if not name:
                    return {'status': 'error', 'message': 'Missing name'}
                success = self.gazebo.delete(name)
                return {'status': 'ok' if success else 'error'}

            elif command == 'gazebo_list_models':
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                models = self.gazebo.list_models()
                return {'status': 'ok', 'models': models}

            elif command == 'gazebo_get_pose':
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                name = message.get('name')
                if not name:
                    return {'status': 'error', 'message': 'Missing name'}
                pose = self.gazebo.get_pose(name)
                if pose:
                    return {'status': 'ok', 'pose': pose}
                return {'status': 'error', 'message': 'Object not found'}

            elif command == 'gazebo_set_pose':
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                name = message.get('name')
                position = message.get('position')
                orientation = message.get('orientation', [0, 0, 0, 1])
                if not name or not position:
                    return {'status': 'error', 'message': 'Missing required parameters'}

                # Create Pose from position/orientation
                from geometry_msgs.msg import Pose, Point, Quaternion
                pose = Pose()
                pose.position = Point(position[0], position[1], position[2])
                pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

                success = self.gazebo.set_pose(name, pose)
                return {'status': 'ok' if success else 'error'}

            elif command == 'gazebo_is_spawned':
                if not self.gazebo:
                    return {'status': 'error', 'message': 'Gazebo not available'}
                name = message.get('name')
                if not name:
                    return {'status': 'error', 'message': 'Missing name'}
                spawned = self.gazebo.is_spawned(name)
                return {'status': 'ok', 'spawned': spawned}


            # ===== Contact Sensor Commands =====
            elif command == 'contact_get_last_landing':
                if not self.contact_sensor:
                    return {'status': 'error', 'message': 'Contact sensor not available'}
                landing = self.contact_sensor.get_last_landing()
                return {'status': 'ok', 'landing': landing}

            elif command == 'contact_start_detection':
                if not self.contact_sensor:
                    return {'status': 'error', 'message': 'Contact sensor not available'}
                self.contact_sensor.enable()
                return {'status': 'ok'}

            elif command == 'contact_stop_detection':
                if not self.contact_sensor:
                    return {'status': 'error', 'message': 'Contact sensor not available'}
                self.contact_sensor.disable()
                return {'status': 'ok'}

            elif command == 'contact_clear':
                if not self.contact_sensor:
                    return {'status': 'error', 'message': 'Contact sensor not available'}
                self.contact_sensor.clear()
                return {'status': 'ok'}


            # ===== RGBD Camera Commands =====
            elif command == 'camera_get_point_cloud':
                if not self.rgbd_camera:
                    return {'status': 'error', 'message': 'RGBD camera not available'}
                cloud = self.rgbd_camera.get_point_cloud()
                if cloud is None:
                    return {'status': 'error', 'message': 'No point cloud data available'}
                return {
                    'status': 'ok',
                    'points': cloud['points'].tolist(),
                    'colors': cloud['colors'].tolist(),
                    'timestamp': cloud['timestamp']
                }

            elif command == 'camera_enable_streaming':
                if not self.rgbd_camera:
                    return {'status': 'error', 'message': 'RGBD camera not available'}
                self.camera_streaming_enabled = True
                return {'status': 'ok'}

            elif command == 'camera_disable_streaming':
                if not self.rgbd_camera:
                    return {'status': 'error', 'message': 'RGBD camera not available'}
                self.camera_streaming_enabled = False
                return {'status': 'ok'}


            # ===== System Commands =====
            elif command == 'get_full_state':
                state = self.get_full_state()
                return {'status': 'ok', 'state': state}

            elif command == 'ping':
                return {'status': 'ok', 'message': 'pong'}

            elif command == 'shutdown':
                self.running = False
                return {'status': 'ok', 'message': 'Shutting down'}

            else:
                return {'status': 'error', 'message': f'Unknown command: {command}'}

        except Exception as e:
            rospy.logerr(f"Command handler error: {e}")
            return {'status': 'error', 'message': str(e)}

    def command_server_loop(self):
        """Main command server loop (blocking)"""
        rospy.loginfo(f"Command server ready (listening on port {self.command_port})")

        while self.running and not rospy.is_shutdown():
            try:
                # Wait for request with timeout
                if self.rep_socket.poll(100):  # 100ms timeout
                    message = self.rep_socket.recv_json()
                    response = self.handle_command(message)
                    self.rep_socket.send_json(response)

            except zmq.Again:
                continue
            except KeyboardInterrupt:
                rospy.loginfo("Received keyboard interrupt")
                self.running = False
                break
            except Exception as e:
                rospy.logerr(f"Command server error: {e}")
                if not self.running:
                    break

        rospy.loginfo("Command server stopped")

    def run(self):
        """Start the server (blocking)"""
        self.print_banner()

        # Start state publisher thread
        pub_thread = threading.Thread(target=self.state_publisher_thread)
        pub_thread.daemon = True
        pub_thread.start()

        try:
            # Run command server (blocking)
            self.command_server_loop()

        except KeyboardInterrupt:
            rospy.loginfo("\nShutting down...")
            self.running = False

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        rospy.loginfo("Cleaning up...")
        self.running = False
        time.sleep(0.5)  # Let threads finish
        self.rep_socket.close()
        self.pub_socket.close()
        self.context.term()
        rospy.loginfo("✓ ZMQ server stopped")

    def print_banner(self):
        """Print startup banner"""
        print("\n" + "="*70)
        print("  Robot ZeroMQ Server - Host-Container Architecture")
        print("="*70)
        print(f"  Command socket (REP):  tcp://*:{self.command_port}")
        print(f"  State broadcast (PUB): tcp://*:{self.state_port}")
        print("\n  Robot Commands:")
        print("    move_robot         - Move to joint positions")
        print("    get_angles         - Get current joint angles")
        print("    get_endpoint_pose  - Get end-effector pose")
        print("    get_robot_state    - Get complete robot state")
        print("\n  Gripper Commands:")
        print("    gripper_open       - Open gripper")
        print("    gripper_close      - Close gripper")
        print("    gripper_set_position - Set gripper position")
        print("    gripper_get_state  - Get gripper state")
        print("\n  System Commands:")
        print("    get_full_state     - Get complete system state")
        print("    ping               - Latency test")
        print("    shutdown           - Stop server")
        print("="*70 + "\n")


if __name__ == '__main__':
    try:
        server = RobotServer(command_port=5555, state_port=5556)
        server.run()
    except rospy.ROSInterruptException:
        pass
