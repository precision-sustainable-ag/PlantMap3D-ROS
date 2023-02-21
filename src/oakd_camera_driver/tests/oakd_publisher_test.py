#!/usr/bin/env python3

import unittest
import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData


class TestImagePublisher(unittest.TestCase):

    def  callback(self):
        
        pass

    def check_camera_data_msg_type():

        """
        Checking what msg type is being published
        """
        pass 

    def check_data_parameters():

        """
        Checking if the topic is publishing the correct data type and shape
        """
        
        pass

if __name__ == '__main__':
    
    pass
    
    