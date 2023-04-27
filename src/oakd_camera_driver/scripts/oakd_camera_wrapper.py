#!/usr/bin/env python3
"""
@author: MathewAaron
"""
"""
    Code for python wrapper
"""
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy 
import subprocess
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

if __name__ == '__main__':
    """
        A camera wrapper for the OakD S2 PoE cameras. 
        This method will parallely start the defined cameras.
    """
    rospy.init_node("camera_wrapper")
    camera_data = [["python3","oakd_capture_data.py","169.254.54.205","camera_1","camera_data"]]#,["python3","oakd_capture_data.py","169.254.54.200","camera_2","camera_data"]]
    processes = []
    for cam_data in camera_data:
        
        process = subprocess.Popen(cam_data)
        processes.append(process)