#!/usr/bin/env python

import rospy
import rospkg
import cv2
import numpy as np
import time
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 

from simulation.srv import Landing as LandingService, LandingRequest, LandingResponse 

class LandingServer:
    def __init__(self):
        rospy.init_node("landing_server")
        
        # --- Internal State ---
        self.bridge = CvBridge()
        self.initial_image = None
        self.current_image = None 
        self.run_id = 0 # Counter for unique file names
        
        # --- File Path Setup ---
        # Create a directory inside the package to save the debug images
        self.log_dir = rospkg.RosPack().get_path('simulation') + "/landing_logs"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo("Created log directory: %s", self.log_dir)

        # --- Topics ---
        self.image_topic = "logi/rgb/image_raw" 
        
        # --- Subscribers ---
        rospy.Subscriber(self.image_topic, Image, self._image_callback)
        rospy.loginfo("Subscribing to Image: %s", self.image_topic)
        
        # --- Service Server ---
        self.service = rospy.Service('get_landing_index', LandingService, self.handle_get_landing_index)
        rospy.loginfo("Landing Index Service ready at: /get_landing_index")

    def _image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def handle_get_landing_index(self, req):
        if self.current_image is None:
            rospy.logwarn("Received request but no current image available.")
            return LandingResponse(index=-1) 

        # 1. START Call (req.is_finish_call == False)
        if not req.is_finish_call:
            # Increment run ID for a new set of images
            self.run_id += 1 
            self.initial_image = self.current_image.copy()
            
            # SAVE INITIAL IMAGE
            file_path = os.path.join(self.log_dir, 'run_{}_A_Initial.png'.format(self.run_id))
            cv2.imwrite(file_path, self.initial_image)
            rospy.loginfo("START: Initial background image stored and saved to %s", file_path)
            
            return LandingResponse(index=-1)

        # 2. FINISH Call (req.is_finish_call == True)
        elif req.is_finish_call:
            if self.initial_image is None:
                rospy.logwarn("FINISH requested, but initial image was not stored. Call START first.")
                return LandingResponse(index=-1)
            
            final_image = self.current_image.copy()
            
            # SAVE FINAL IMAGE
            file_path = os.path.join(self.log_dir, 'run_{}_B_Final.png'.format(self.run_id))
            cv2.imwrite(file_path, final_image)

            index = self._calculate_landing_index(self.initial_image, final_image)
            
            self.initial_image = None
            
            rospy.loginfo("FINISH: Calculated index: %d. Images saved to %s. Resetting for next cycle.", index, self.log_dir)
            return LandingResponse(index)
        
        return LandingResponse(index=-1)


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
        server = LandingServer()
        server.start()
    except rospy.ROSInterruptException:
        pass