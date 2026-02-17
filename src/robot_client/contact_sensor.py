"""
ContactSensorClient - Contact and landing detection

Interfaces with contact sensors in Gazebo to detect object landings.
"""

from .base_client import BaseClient


class ContactSensorClient(BaseClient):
    """
    Contact sensor client for landing detection.
    
    Provides access to collision/contact events from Gazebo sensors.
    Can detect when and where objects land on surfaces.
    """
    
    def get_last_landing(self):
        """
        Get the last detected landing event.
        
        Returns:
            dict: Landing data with keys:
                - position: [x, y, z] coordinates
                - object_name: Name of landed object
                - timestamp: Time of landing (seconds)
            None if no landing detected
        """
        result = self._client.contact_get_last_landing()
        return result.get('landing') if result else None
    
    def start_detection(self):
        """
        Enable contact detection monitoring.
        
        Returns:
            bool: True if successful
        """
        return self._client.contact_start_detection()
    
    def stop_detection(self):
        """
        Disable contact detection monitoring.
        
        Returns:
            bool: True if successful
        """
        return self._client.contact_stop_detection()
    
    def clear(self):
        """
        Clear all landing data.
        
        Returns:
            bool: True if successful
        """
        return self._client.contact_clear()
    
    def wait_for_landing(self, timeout=5.0, poll_rate=50):
        """
        Wait for a landing event to be detected.
        
        Polls the sensor state until landing is detected or timeout occurs.
        
        Args:
            timeout: Maximum time to wait in seconds
            poll_rate: Polling frequency in Hz
        
        Returns:
            dict: Landing data (same as get_last_landing)
            None if timeout occurs
        """
        import time
        start_time = time.time()
        sleep_time = 1.0 / poll_rate
        
        while (time.time() - start_time) < timeout:
            landing = self.get_last_landing()
            if landing is not None:
                return landing
            time.sleep(sleep_time)
        
        return None
