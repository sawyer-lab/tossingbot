#!/usr/bin/env python3
"""
Health Monitor for TossingBot Environment
Detects failures and automatically restarts Gazebo when needed.
"""
import rospy
import subprocess
import time
import signal
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

class HealthMonitor:
    def __init__(self):
        self.last_joint_state_time = None
        self.last_camera_time = None
        self.joint_state_timeout = 5.0  # seconds
        self.step_timeout = 30.0  # Maximum time for single grasp attempt
        self.last_step_start = None
        
        # Subscribe to joint states to monitor connection
        rospy.Subscriber('/robot/joint_states', JointState, self._joint_state_callback)
        
        # Services for Gazebo control
        self.reset_sim_srv = None
        try:
            rospy.wait_for_service('/gazebo/reset_simulation', timeout=2.0)
            self.reset_sim_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        except:
            rospy.logwarn("Health Monitor: /gazebo/reset_simulation service not available")
    
    def _joint_state_callback(self, msg):
        """Update last time we received joint states"""
        self.last_joint_state_time = rospy.Time.now()
    
    def mark_step_start(self):
        """Call this at the beginning of each grasp attempt"""
        self.last_step_start = rospy.Time.now()
    
    def is_healthy(self):
        """
        Check if the environment is healthy.
        Returns: (is_healthy: bool, reason: str)
        """
        now = rospy.Time.now()
        
        # Check 1: Are we receiving joint states?
        if self.last_joint_state_time is None:
            return False, "Never received joint states"
        
        joint_age = (now - self.last_joint_state_time).to_sec()
        if joint_age > self.joint_state_timeout:
            return False, f"Joint states stale ({joint_age:.1f}s old)"
        
        # Check 2: Is current step taking too long?
        if self.last_step_start is not None:
            step_duration = (now - self.last_step_start).to_sec()
            if step_duration > self.step_timeout:
                return False, f"Step timeout ({step_duration:.1f}s)"
        
        return True, "OK"
    
    def try_soft_reset(self):
        """
        Try to reset Gazebo simulation without killing it.
        Returns: success (bool)
        """
        if self.reset_sim_srv is None:
            return False
        
        try:
            rospy.logwarn("Health Monitor: Attempting soft reset via /gazebo/reset_simulation")
            self.reset_sim_srv()
            rospy.sleep(2.0)
            return True
        except Exception as e:
            rospy.logerr(f"Health Monitor: Soft reset failed: {e}")
            return False
    
    def restart_gazebo(self):
        """
        Nuclear option: Kill and restart Gazebo + robot controllers.
        This should be called as a last resort.
        """
        rospy.logerr("="*70)
        rospy.logerr("Health Monitor: RESTARTING GAZEBO (Environment Failure Detected)")
        rospy.logerr("="*70)
        
        try:
            # Kill gazebo processes using pkill
            rospy.logwarn("Killing Gazebo processes...")
            subprocess.run(['pkill', '-9', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', 'gzclient'], stderr=subprocess.DEVNULL)
            
            time.sleep(3.0)
            
            # Restart via roslaunch
            rospy.logwarn("Restarting Gazebo...")
            subprocess.Popen([
                'roslaunch', 
                'custom_sawyer_gazebo', 
                'spawn_robot.launch'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Wait for gazebo to come back online
            time.sleep(10.0)
            
            rospy.loginfo("Health Monitor: Gazebo restart complete")
            
            # Reset our tracking
            self.last_joint_state_time = None
            self.last_step_start = None
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Health Monitor: Failed to restart Gazebo: {e}")
            return False
    
    def check_and_recover(self):
        """
        Check health and attempt recovery if needed.
        Returns: (recovered: bool, reason: str)
        """
        is_healthy, reason = self.is_healthy()
        
        if is_healthy:
            return True, "Healthy"
        
        rospy.logwarn(f"Health Monitor: Environment unhealthy - {reason}")
        
        # Try soft reset first
        if self.try_soft_reset():
            # Give it a moment and check again
            rospy.sleep(2.0)
            is_healthy, _ = self.is_healthy()
            if is_healthy:
                rospy.loginfo("Health Monitor: Soft reset successful")
                return True, "Recovered via soft reset"
        
        # Soft reset didn't work, try hard restart
        if self.restart_gazebo():
            return True, "Recovered via Gazebo restart"
        
        return False, f"Recovery failed - {reason}"
