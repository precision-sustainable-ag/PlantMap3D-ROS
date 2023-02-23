#!/usr/bin/env python3

import unittest
import rostest
import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData

import numpy as np

class TestImagePublisher(unittest.TestCase):

    def __init__(self,*args):

        super(TestImagePublisher,self).__init__(*args)
        # initializing dummy camera data
        rospy.init_node("oakd_camera_test")
        self.rgb_data = np.random.randint(0,255,(1024,1024,3),dtype=np.int64)
        self.depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        self.segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)
        self.message_data = []

        self.pub = rospy.Publisher('camera_data_test',numpy_msg(PM3DCameraData),queue_size=2)
       
    
    def callback(self,data):
        rospy.loginfo("Appending Camera Data")
        self.message_data.append(data)
    
    def image_publisher(self):

        rate = rospy.Rate(1)
        count = 0
        data = PM3DCameraData()
        data.rgb_data = self.rgb_data.flatten().tolist()
        data.depth_map = self.depth_data.flatten().tolist()
        data.segmentation_labels = self.segmentation_label_arr.flatten().tolist()
        data.rgb_dims = self.rgb_data.shape
        data.depth_map_dims = self.depth_data.shape
        data.segmentation_label_dims = self.segmentation_label_arr.shape
        rospy.Subscriber('camera_data_test',numpy_msg(PM3DCameraData),callback=self.callback)
        # publishing data 5 times
        while count < 6:
            rospy.loginfo("Publishing Camera Data")
            self.pub(data)
            count += 1
            rate.sleep()
        self.assertGreater(len(self.message_data),0,'No messages received')


    def check_camera_data_msg_type(self):

        """
        Checking what msg type is being published
        """
        for data in self.message_data:
            self.assertEqual(type(data),'PM3DCameraData')


    def check_data_parameters(self):

        """
        Checking if the topic is publishing the correct data type and shape
        """
        
        self.assertTrue(self.message_data)

if __name__ == '__main__':
    
    rostest.rosrun('oakd_camera_driver','oakd_camera_test',TestImagePublisher)
    
    