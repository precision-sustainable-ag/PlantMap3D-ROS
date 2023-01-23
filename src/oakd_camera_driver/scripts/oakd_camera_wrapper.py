#!/usr/bin/env python3

"""
    Code for python wrapper
"""

from oakd_capture_rgb import StartCameraStream
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import numpy as np

class MultiCameraDriver():

    def __init__(self):
        pass
    
    def get_camera_list(self):
        pass

    def start_stream(self):
        pass

if __name__ == '__main__':
	
    # initiating ros node
    rospy.init_node("camera_driver")

    # enter IP address of the cameras in the input field.
    cmd_cam = StartCameraStream("")
    cmd_cam.run_camera()