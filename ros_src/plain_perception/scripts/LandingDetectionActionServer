#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 

from plain_perception.msg import LandingDetectionAction, LandingDetectionResult, LandingDetectionFeedback

class LandingActionServer:
    def __init__(self):
        rospy.init_node("landing_action_server")
        
        # --- Internal State ---
        self.bridge = CvBridge()
        self.current_image = None 
        self.run_id = 0
        
        # --- File Path Setup ---
        self.log_dir = rospkg.RosPack().get_path('simulation') + "/landing_logs"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo("Created log directory: %s", self.log_dir)

        # --- Topics ---
        self.image_topic = "logi/rgb/image_raw" 
        
        # --- Subscribers ---
        rospy.Subscriber(self.image_topic, Image, self._image_callback)
        rospy.loginfo("Subscribing to Image: %s", self.image_topic)
        
        # --- Action Server ---
        self.action_server = actionlib.SimpleActionServer(
            'landing_detection',
            LandingDetectionAction,
            execute_cb=self.execute_detection,
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Landing Detection Action Server ready")

    def _image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def execute_detection(self, goal):
        """
        Execute the landing detection action.
        Goal contains wait_duration (how long to wait before taking final image)
        """
        rate = rospy.Rate(10)  # 10Hz feedback rate
        feedback = LandingDetectionFeedback()
        result = LandingDetectionResult()
        
        # Check if we have an image
        if self.current_image is None:
            result.landing_index = -1
            result.message = "No image available"
            self.action_server.set_aborted(result)
            return
        
        # Increment run ID
        self.run_id += 1
        
        # Capture initial image
        initial_image = self.current_image.copy()
        file_path = os.path.join(self.log_dir, 'run_{}_A_Initial.png'.format(self.run_id))
        cv2.imwrite(file_path, initial_image)
        rospy.loginfo("Captured initial image, saved to %s", file_path)
        
        # Wait for specified duration with feedback
        wait_duration = rospy.Duration(5.0)  # Default 5 seconds, or use goal.wait_duration
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time) < wait_duration:
            # Check if action was preempted (cancelled)
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Landing detection preempted")
                self.action_server.set_preempted()
                return
            
            # Send feedback
            feedback.elapsed_time = (rospy.Time.now() - start_time).to_sec()
            feedback.status = "Waiting for drone to land... {:.1f}s".format(feedback.elapsed_time)
            self.action_server.publish_feedback(feedback)
            
            rate.sleep()
        
        # Capture final image
        if self.current_image is None:
            result.landing_index = -1
            result.message = "Lost image stream during detection"
            self.action_server.set_aborted(result)
            return
            
        final_image = self.current_image.copy()
        file_path = os.path.join(self.log_dir, 'run_{}_B_Final.png'.format(self.run_id))
        cv2.imwrite(file_path, final_image)
        
        # Calculate landing index
        landing_index = self._calculate_landing_index(initial_image, final_image)
        
        # Set result
        result.landing_index = landing_index
        if landing_index >= 0:
            result.message = "Landing detected at x={}".format(landing_index)
            self.action_server.set_succeeded(result)
            rospy.loginfo("Landing detection succeeded: %s", result.message)
        else:
            result.message = "No landing detected"
            self.action_server.set_aborted(result)
            rospy.logwarn("Landing detection failed")

    def _calculate_landing_index(self, initial_img, landing_img):
        if initial_img.shape != landing_img.shape:
            rospy.logerr("Images have different sizes, cannot compare.")
            return -1

        initial_gray = cv2.cvtColor(initial_img, cv2.COLOR_BGR2GRAY)
        landing_gray = cv2.cvtColor(landing_img, cv2.COLOR_BGR2GRAY)
        difference = cv2.absdiff(initial_gray, landing_gray)
        _, thresholded = cv2.threshold(difference, 50, 255, cv2.THRESH_BINARY)
        
        # SAVE DIFFERENCE IMAGE
        diff_file_path = os.path.join(self.log_dir, 'run_{}_C_Difference.png'.format(self.run_id))
        cv2.imwrite(diff_file_path, thresholded)

        contours_info = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[-2] 
        
        if not contours:
            rospy.loginfo("No significant object difference detected.")
            return -1

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        
        if M["m00"] == 0:
             return -1

        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])

        # Create and SAVE CONTOUR IMAGE
        debug_img = landing_img.copy()
        cv2.drawContours(debug_img, [largest_contour], -1, (0, 255, 0), 2) 
        cv2.circle(debug_img, (center_x, center_y), 5, (0, 0, 255), -1)    
        cv2.putText(debug_img, 'Index: {}'.format(center_x), (center_x + 10, center_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        contour_file_path = os.path.join(self.log_dir, 'run_{}_D_Contour_{}.png'.format(self.run_id, center_x))
        cv2.imwrite(contour_file_path, debug_img)

        return center_x

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        server = LandingActionServer()
        server.start()
    except rospy.ROSInterruptException:
        pass