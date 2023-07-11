#!/usr/bin/env python3
"""
@author: MathewAaron

NOTE : TO RUN THIS FILE

    PlantMap3D/ 
        catkin_make run_tests_oakd_camera_driver
"""
from unittest import TestCase
import cv2
import rostest
import os 
import time
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import rospkg
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import String
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np
np.random.seed(45)


class TestImageSubscriber(TestCase):

    def __init__(self, *args) :
        super(TestImageSubscriber,self).__init__(*args)
        
        self.success = False
        self.bridge = CvBridge() 

        rospack_testset = rospkg.RosPack() 
        
        self.__test_rgbdatapath = rospack_testset.get_path('oakd_camera_driver') + '/test/rgb/1674584672.191502.jpg'
        self.__test_depthdatapath = rospack_testset.get_path('oakd_camera_driver') + '/test/depth/1_Depth.png'

        self.__test_segmentationpath = rospack_testset.get_path('oakd_camera_driver') + '/test/segmentation/1_Seg.jpg'
        rospy.init_node('camera_data_test')
        self.subscribed_depth_data = None
        self.subscribed_rgb_data = None 
        self.subscribed_segmentation_labels = None

        self.sub = rospy.Subscriber("camera_1",numpy_msg(PM3DCameraData),callback=self.callback)
        self.pub = rospy.Publisher("camera_1",numpy_msg(PM3DCameraData),queue_size=3)


    def callback(self,data):
        rospy.loginfo("Appending Camera Data")
        
        self.subscribed_rgb_data = data.rgb_data 
        self.subscribed_depth_data = data.depth_map 
        self.subscribed_segmentation_labels = data.segmentation_labels
        self.success = True 
    


    def test_check_rgb_data(self):

        """
            Testing RGB data that is published
        """
        rgb_msg = PM3DCameraData()
        

        rgb_data = cv2.imread(self.__test_rgbdatapath)

        rgb_msg.rgb_data = self.bridge.cv2_to_imgmsg(rgb_data,"bgr8")
        
        rospy.sleep(1)
        self.pub.publish(rgb_msg)
        rospy.sleep(1)
        test_rgb_data = self.bridge.imgmsg_to_cv2(self.subscribed_rgb_data)

        # checking rgb
        assert((test_rgb_data == rgb_data).all()), "Arrays are not same"

    def test_check_depth_data(self):

        """
            Testing Depth data that is published
        """
        depth_msg = PM3DCameraData()
        depthdata = cv2.imread(self.__test_depthdatapath)
        
        depth_msg.depth_map = self.bridge.cv2_to_imgmsg(depthdata,"bgr8") 

        rospy.sleep(1)
        self.pub.publish(depth_msg)
        rospy.sleep(1)

        test_data = self.bridge.imgmsg_to_cv2(self.subscribed_depth_data)
        assert((test_data == depthdata).all()), "Arrays are not same"

    def test_check_segmentation_data(self):

        """
            Testing Segmentation data that is published
        """
        
        segmentation_msg = PM3DCameraData()
        segmentation_data = cv2.imread(self.__test_segmentationpath)
        
        segmentation_msg.segmentation_labels = self.bridge.cv2_to_imgmsg(segmentation_data,"bgr8") 

        rospy.sleep(1)
        self.pub.publish(segmentation_msg)
        rospy.sleep(1)

        test_data = self.bridge.imgmsg_to_cv2(self.subscribed_segmentation_labels)
        assert((test_data == segmentation_data).all()), "Arrays are not same"
        


if __name__ == '__main__':    
    rostest.rosrun('oakd_camera_driver','test_oakd_publisher.py',TestImageSubscriber)
    
    