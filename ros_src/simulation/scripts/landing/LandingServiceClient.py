#!/usr/bin/env python

import rospy
from simulation.srv import Landing as LandingService, LandingRequest, LandingResponse 

class LandingClient:
    def __init__(self, service_name='get_landing_index'):
        self.service_name = service_name
        rospy.loginfo("Waiting for Landing Server Service: %s", self.service_name)
      
        rospy.wait_for_service(self.service_name)
        self.proxy = rospy.ServiceProxy(self.service_name, LandingService)
        rospy.loginfo("Landing Client ready.")

    def start(self):
        rospy.loginfo("Sending START command to Landing Server...")
        try:
            req = LandingRequest(is_finish_call=False)
            response = self.proxy(req)
            
            if response.index == -1:
                rospy.loginfo("Server successfully stored initial image.")
                return True
            else:
                rospy.logwarn("Server returned an unexpected index during START.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False

    def finish(self):
        rospy.loginfo("Sending FINISH command to Landing Server...")
        try:
            req = LandingRequest(is_finish_call=True)
            response = self.proxy(req)
            
            if response.index == -1:
                rospy.logwarn("Server failed to calculate the index (e.g., initial image missing or no object found).")
            else:
                rospy.loginfo("Received landing index: %d", response.index)
            
            return response.index
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return -1

if __name__ == '__main__':
    rospy.init_node('landing_client_tester', anonymous=True)
    
    client = LandingClient()
    
    rospy.spin()