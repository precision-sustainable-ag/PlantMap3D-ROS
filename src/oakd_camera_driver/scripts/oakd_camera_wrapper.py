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


if __name__ == '__main__':
	
    camera_data = [["169.254.54.205","camera_1"],["169.254.54.205","camera_2"]]
    
