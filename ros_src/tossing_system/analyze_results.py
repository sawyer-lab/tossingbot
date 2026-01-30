#!/usr/bin/env python
from __future__ import print_function
import csv
import numpy as np
import sys
import os

def analyze_results(filename):
    if not os.path.exists(filename):
        print("Error: File '{}' not found.".format(filename))
        return

    data = {}
    
    print("Analyzing {}...\n".format(filename))

    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            speed = float(row['Speed'])
            success = row['Success'].strip() == 'True'
            
            if speed not in data:
                data[speed] = {'x': [], 'y': [], 'success_count': 0, 'total': 0}
            
            data[speed]['total'] += 1
            if success:
                try:
                    lx = float(row['Landing_X'])
                    ly = float(row['Landing_Y'])
                    data[speed]['x'].append(lx)
                    data[speed]['y'].append(ly)
                    data[speed]['success_count'] += 1
                except ValueError:
                    pass # Skip bad data

    # Print Header
    headers = [
        "Speed (m/s)", "Total", "Success%", 
        "Mean X", "Std X", "Med X", 
        "Mean Y", "Std Y", "Med Y"
    ]
    print("{:<12} | {:<6} | {:<9} | {:<8} | {:<8} | {:<8} | {:<8} | {:<8} | {:<8}".format(*headers))
    print("-" * 100)

    speeds = sorted(data.keys())
    for s in speeds:
        stats = data[s]
        total = stats['total']
        succ = stats['success_count']
        rate = (succ / float(total)) * 100 if total > 0 else 0.0
        
        if succ > 0:
            x_arr = np.array(stats['x'])
            y_arr = np.array(stats['y'])
            
            mean_x, std_x, med_x = np.mean(x_arr), np.std(x_arr), np.median(x_arr)
            mean_y, std_y, med_y = np.mean(y_arr), np.std(y_arr), np.median(y_arr)
        else:
            mean_x = std_x = med_x = 0.0
            mean_y = std_y = med_y = 0.0

        print("{:<12.1f} | {:<6} | {:<8.1f}% | {:<8.3f} | {:<8.3f} | {:<8.3f} | {:<8.3f} | {:<8.3f} | {:<8.3f}".format(
            s, total, rate, mean_x, std_x, med_x, mean_y, std_y, med_y))

if __name__ == "__main__":
    target_file = "toss_results.csv"
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    
    # Try to find the file in common locations if not in CWD
    if not os.path.exists(target_file):
        # Check script dir
        script_dir = os.path.dirname(os.path.abspath(__file__))
        possible_path = os.path.join(script_dir, "src/tossingbot/scripts", target_file)
        if os.path.exists(possible_path):
            target_file = possible_path
        else:
            # Check workspace root assumption
            possible_path = os.path.expanduser("~/ros_ws/" + target_file)
            if os.path.exists(possible_path):
                target_file = possible_path

    analyze_results(target_file)
