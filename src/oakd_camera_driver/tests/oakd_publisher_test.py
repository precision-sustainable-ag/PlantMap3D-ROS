#!/usr/bin/env python3

import unittest
import rostest
import os 
import queue
import threading
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData

import numpy as np

from test_image_publisher import TestImagePublisher

class TestImageSubscriber(unittest.TestCase):

    def __init__(self,*args):

        super(TestImagePublisher,self).__init__(*args)
        # initializing dummy camera data
        # rospy.init_node("oakd_camera_test")
        image_publisher = TestImagePublisher()
        self.queue = queue.Queue()
        self.publisher_thread = threading.Thread(target=image_publisher.run())
        self.subscriber_thread = threading.Thread(target=self.image_subscriber())
        
        self.array_data_flag, self.depth_data_flag, self.segmentation_label_arr_flag = image_publisher.return_current_camera_data()
        
        self.subscribed_rgb_data = None
        self.subscribed_depth_data = None 
        self.subscribed_segmentation_labels = None 
        
        self.subscribed_rgb_data_dims = None 
        self.subscribed_depth_data_dims = None 
        self.subscribed_segmentation_data_dims = None

        self.subscribed_data = None

        self.publisher_thread.start()
        self.subscriber_thread.start()
        self.publisher_thread.join()
        self.subscriber_thread.join()
       
    
    def image_subscriber(self):

        rospy.init_node("camera_data_subscriber")
        rospy.Subscriber("camera_1",numpy_msg(PM3DCameraData),callback=self.callback)
        rospy.spin()
    
    def callback(self,data):
        rospy.loginfo("Appending Camera Data")
        self.subscribed_rgb_data = data.rgb_data
        self.subscribed_depth_data = data.depth_map
        self.subscribed_segmentation_labels = data.segmentation_labels
        self.subscribed_rgb_data_dims = data.rgb_dims
        self.subscribed_depth_data_dims = data.depth_map_dims
        self.subscribed_segmentation_data_dims = data.segmentation_label_dims

        self.subscribed_data = data


    def test_check_camera_data_msg_type(self):

        """
        Checking what msg type is being published
        """
        
        self.assertEqual(type(self.data),'PM3DCameraData')


    def test_check_data_parameters(self):

        """
        Checking if the topic is publishing the correct data type and shape
        """
        
        subscribed_rgb_data = self.subscribed_rgb_data.reshape((self.subscribed_rgb_data_dims))
        assert ((subscribed_rgb_data == self.array_data_flag).all())
        
if __name__ == '__main__':
    
    rostest.rosrun('oakd_camera_driver','oakd_camera_test',TestImageSubscriber)
    
    