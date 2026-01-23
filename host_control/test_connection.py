#!/usr/bin/env python3
"""
Simple test to send commands to the Sawyer robot in the Docker container.
Run this on the HOST (Ubuntu 24) while the Docker container is running.
"""
import roslibpy
import time

# Connect to ROS bridge in Docker container
client = roslibpy.Ros(host='localhost', port=9090)

try:
    client.run()
    print("✓ Connected to ROS bridge!")
    
    # Joint names for Sawyer
    joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 
                   'right_j4', 'right_j5', 'right_j6']
    
    # Create publisher for joint commands
    joint_command_pub = roslibpy.Topic(
        client, 
        '/robot/limb/right/joint_command',
        'intera_core_msgs/JointCommand'
    )
    
    # Subscribe to joint states for feedback
    def joint_state_callback(msg):
        # Find sawyer joints in the message
        if 'right_j0' in msg['name']:
            indices = [msg['name'].index(n) for n in joint_names]
            positions = [msg['position'][i] for i in indices]
            print(f"Current positions: {[f'{p:.3f}' for p in positions]}")
    
    joint_state_sub = roslibpy.Topic(
        client,
        '/robot/joint_states',
        'sensor_msgs/JointState'
    )
    joint_state_sub.subscribe(joint_state_callback)
    
    print("✓ Subscribed to joint states")
    print("\nWaiting 2 seconds for initial state...")
    time.sleep(2)
    
    # Send a simple position command
    print("\n✓ Sending test joint command (all zeros)...")
    command = {
        'mode': 1,  # POSITION_MODE
        'names': joint_names,
        'position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'header': {
            'stamp': {
                'secs': int(time.time()),
                'nsecs': int((time.time() % 1) * 1e9)
            }
        }
    }
    
    joint_command_pub.publish(roslibpy.Message(command))
    print("✓ Command sent!")
    
    print("\nMonitoring for 5 seconds...")
    time.sleep(5)
    
except roslibpy.exceptions.RosLibPyException as e:
    print(f"✗ Connection failed: {e}")
    print("\nMake sure:")
    print("  1. Docker container is running")
    print("  2. rosbridge_websocket is running in the container")
    print("     Run: docker exec -it robo2025 bash -c 'roslaunch rosbridge_server rosbridge_websocket.launch'")
finally:
    client.terminate()
    print("\n✓ Disconnected")
