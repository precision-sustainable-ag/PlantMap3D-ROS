#!/usr/bin/env python3

import unittest
import rostest
import os 
import time
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import String

import numpy as np
np.random.seed(45)
from test_image_publisher import TestImagePublisher

class TestImageSubscriber(unittest.TestCase):

    def setUp(self):

        # initializing dummy camera data
        # rospy.init_node("oakd_camera_test")
        rospy.init_node("camera_data_subscriber")
        self.rgb_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        self.depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        self.segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)
        
        self.subscribed_rgb_data = None
        self.subscribed_depth_data = None 
        self.subscribed_segmentation_labels = None 
        
        self.subscribed_rgb_data_dims = None 
        self.subscribed_depth_data_dims = None 
        self.subscribed_segmentation_data_dims = None

        self.subscribed_data = None

        self.success = False
        self.sub = rospy.Subscriber("camera_1",numpy_msg(PM3DCameraData),callback=self.callback)
       
        self.msg = PM3DCameraData()

    def tearDown(self):
        self.sub.unregister()
   
    def callback(self,data):
        rospy.loginfo("Appending Camera Data")
        self.subscribed_rgb_data = data.rgb_data
        self.subscribed_depth_data = data.depth_map
        self.subscribed_segmentation_labels = data.segmentation_labels
        self.subscribed_rgb_data_dims = data.rgb_dims
        self.subscribed_depth_data_dims = data.depth_map_dims
        self.subscribed_segmentation_data_dims = data.segmentation_label_dims

        
        self.subscribed_data = data
        self.success = True
        


    def test_check_rgb_data(self):

        """
        Checking what msg type is being published
        """
        self.msg.rgb_data = self.rgb_data.flatten().tolist()

        self.msg.rgb_dims = self.rgb_data.shape

        pub = rospy.Publisher("camera_1",numpy_msg(PM3DCameraData),queue_size=10)
        rospy.sleep(1)
        pub.publish(self.msg)
        rospy.sleep(1)
        test_data = self.subscribed_rgb_data.reshape((self.subscribed_rgb_data_dims))
        rospy.loginfo(f"The variable is  {self.subscribed_rgb_data}")
        assert((test_data == self.rgb_data).all()), "Arrays are not same"

    def test_check_depth_data(self):

        """
        Checking what msg type is being published
        """
        
        self.msg.depth_map = self.depth_data.flatten().tolist()
        
        self.msg.depth_map_dims = self.depth_data.shape
        pub = rospy.Publisher("camera_1",numpy_msg(PM3DCameraData),queue_size=10)
        rospy.sleep(1)
        pub.publish(self.msg)
        rospy.sleep(1)
        test_data = self.subscribed_depth_data.reshape((self.subscribed_depth_data_dims))
        rospy.loginfo(f"The variable is  {self.subscribed_rgb_data}")
        assert((test_data == self.depth_data).all()), "Arrays are not same"

    def test_check_segmentation_data(self):

        """
        Checking what msg type is being published
        """
        
        self.msg.segmentation_labels = self.segmentation_label_arr.flatten().tolist()
        
        self.msg.segmentation_label_dims = self.segmentation_label_arr.shape
        pub = rospy.Publisher("camera_1",numpy_msg(PM3DCameraData),queue_size=10)
        rospy.sleep(1)
        pub.publish(self.msg)
        rospy.sleep(1)
        test_data =self.subscribed_segmentation_labels.reshape((self.subscribed_segmentation_data_dims))
        rospy.loginfo(f"The variable is  {self.subscribed_rgb_data}")
        assert((test_data == self.segmentation_label_arr ).all()), "Arrays are not same"
        

        


if __name__ == '__main__':
    
    rostest.rosrun('oakd_camera_driver','oakd_camera_test',TestImageSubscriber)
    
    