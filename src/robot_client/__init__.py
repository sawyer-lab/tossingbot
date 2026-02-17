"""
Robot Client Library

Simple Python client for controlling robots through the container API.
Supports multiple communication protocols: ZMQ, HTTP, WebSocket.

Each subsystem has its own client class:
- RobotClient: Robot arm control
- GripperClient: Gripper control
- CameraClient: Camera control
- GazeboClient: Gazebo simulation control
- HeadClient: Head pan and display
- LightsClient: Robot lights control
- ContactSensorClient: Landing detection
- RGBDCameraClient: Point cloud capture
"""

from .robot import RobotClient
from .gripper import GripperClient
from .camera import CameraClient
from .gazebo import GazeboClient
from .head import HeadClient
from .lights import LightsClient
from .contact_sensor import ContactSensorClient
from .rgbd_camera import RGBDCameraClient

__version__ = "1.0.0"
__all__ = [
    "RobotClient",
    "GripperClient",
    "CameraClient",
    "GazeboClient",
    "HeadClient",
    "LightsClient",
    "ContactSensorClient",
    "RGBDCameraClient",
]
