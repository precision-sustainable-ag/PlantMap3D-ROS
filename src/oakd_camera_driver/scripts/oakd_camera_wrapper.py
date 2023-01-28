#!/usr/bin/env python3

"""
    Code for python wrapper
"""

from oakd_capture_rgb import StartCameraStream
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Bool 
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

if __name__ == '__main__':
	
    camera_data = [["169.254.54.205","camera_1","camera_1_data"],["169.254.54.200","camera_2","camera_2_data"]]

    for cam_data in camera_data:
        
        commd = "python3 oakd_capture_rgb.py {} {} {} &".format(str(cam_data[0]),str(cam_data[1]),str(cam_data[2]))
        os.system(commd)