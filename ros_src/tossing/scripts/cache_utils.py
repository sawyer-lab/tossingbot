import os
import pickle
import numpy as np

class SimpleTrajectoryCache(object):
    def __init__(self, clear_on_start=True):
        self.cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "3r_cache")
        
        # Ensure dir exists
        if not os.path.exists(self.cache_dir):
            os.makedirs(self.cache_dir)
        elif clear_on_start:
            # Clear old cache files on startup if requested
            self.clear_cache()

    def get_key(self, speed):
        """
        Creates a simple filename based on scalar speed.
        We round to 3 decimal places (e.g., 2.123 m/s).
        Example: speed_2.123.pkl
        """
        # Ensure it is a float and round it
        speed_val = float(speed)
        key_str = "speed_{:.3f}".format(speed_val)
        return os.path.join(self.cache_dir, key_str + ".pkl")

    def save(self, speed, solution):
        """
        Saves the solution for a specific speed.
        """
        filepath = self.get_key(speed)
        
        # We only save what we need to execute
        data = {
            "Q": solution["Q"],
            "Qd": solution["Qd"],
            "Qdd": solution["Qdd"],
            "time": solution["time"]
        }
        
        try:
            with open(filepath, "wb") as f:
                pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            print("Warning: Failed to save cache: {}".format(e))

    def load(self, speed):
        """
        Loads the solution if this speed (rounded to 3 decimals) exists.
        """
        filepath = self.get_key(speed)
        
        if not os.path.exists(filepath):
            return None
            
        try:
            with open(filepath, "rb") as f:
                data = pickle.load(f)
            # print("Loaded trajectory from cache: {}".format(filepath))
            return data
        except Exception as e:
            print("Warning: Cache file corrupted, ignoring: {}".format(e))
            return None

    def clear_cache(self):
        folder = self.cache_dir
        if not os.path.exists(folder):
            return

        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print('Failed to delete {}. Reason: {}'.format(file_path, e))