#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_TOPIC_MAP = {
    'head': 'head_camera',
    'hand': 'right_hand_camera',
}


class Camera:
    def __init__(self, camera_name='head_camera'):
        self.camera_name = camera_name
        self.bridge = CvBridge()
        self.current_image = None
        self._streaming = False

        topic = f"/io/internal_camera/{camera_name}/image_raw"
        self.image_sub = rospy.Subscriber(
            topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.sleep(0.5)
        rospy.loginfo(f"Camera: {camera_name} ready, subscribed to {topic}")

    def start_streaming(self):
        self._streaming = True
        return True

    def stop_streaming(self):
        self._streaming = False
        return True

    def get_state(self):
        return {
            'camera': self.camera_name,
            'streaming': self._streaming,
            'has_image': self.current_image is not None,
            'topic': f"/io/internal_camera/{self.camera_name}/image_raw",
        }

    def get_image(self):
        if self.current_image is not None:
            return self.current_image.copy()
        else:
            rospy.logwarn_throttle(2.0, "Camera: No image received yet")
            return None

    def get_image_compressed(self):
        img = self.get_image()
        if img is not None:
            _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buffer.tobytes()
        return None

    def _image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Camera: Failed to convert image: {e}")


if __name__ == "__main__":
    rospy.init_node("camera_test")

    cam = Camera("head_camera")
    rospy.sleep(2.0)

    img = cam.get_image()
    if img is not None:
        print(f"Got image: {img.shape}")
        cv2.imwrite("/tmp/test_camera.jpg", img)
        print("Saved to /tmp/test_camera.jpg")
    else:
        print("No image available")
