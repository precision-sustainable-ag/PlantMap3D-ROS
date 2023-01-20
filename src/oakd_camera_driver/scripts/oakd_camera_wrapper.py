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

class MultiCameraDriver():

    def __init__(self,ip_1):
        self.ip_1 = ""
        self.gps_flag = False
        self.cmd_cam = StartCameraStream(self.ip_1,self.gps_flag)

    def get_camera_list(self):
        pass

    def start_stream(self):
        self.cmd_cam.start_camera_stream()

    def get_gps_message(self):
        """
        Subscribed to GPS data

            # Need to check activity i.e. how long is this subscriber active / node is running
        """
        rospy.loginfo('Getting Data From GPS')
        self.gps_flag = rospy.Subscriber("/gps_data",Bool)
        rospy.spin()

    # def get_image_frame(self):
    #     self.get_gps_message()
    #     self.cmd_cam.start_camera_stream(self.gps_flag) 


if __name__ == '__main__':
	
    # initiating ros node
    rospy.init_node("camera_driver")
    test_node = MultiCameraDriver("169.254.54.205")
    test_node.get_image_frame()