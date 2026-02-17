#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HeadDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        rospy.loginfo("HeadDisplay: Initialized")

    def display_image(self, image_path=None, cv_image=None):
        if image_path:
            img = cv2.imread(image_path)
            if img is None:
                rospy.logerr(f"Cannot read image at '{image_path}'")
                return False
        elif cv_image is not None:
            img = cv_image
        else:
            rospy.logerr("Must provide either image_path or cv_image")
            return False

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self._image_pub.publish(msg)
        return True

    def clear(self):
        blank = cv2.imread('/dev/null') if False else (0 * cv2.ones((600, 1024, 3), dtype='uint8'))
        msg = self.bridge.cv2_to_imgmsg(blank, encoding="bgr8")
        self._image_pub.publish(msg)
        return True
