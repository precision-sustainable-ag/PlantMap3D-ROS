#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import matplotlib.pyplot as plt
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ImageReader():
    """
        This program will act as a trigger from the GPS interpreter module, 
        the function will collect all the images from the camera nodes and then publish them
    """
    def __init__(self,node_name):
        
        rospy.init_node(node_name)
        self.camera_image_list = []
        self.camera1_image = Subscriber('/camera_1_data',Image)
        self.camera2_image = Subscriber('/camera_2_data',Image)
        self.br = CvBridge()
        self.tss = ApproximateTimeSynchronizer([self.camera1_image,self.camera2_image],2,0.2,allow_headerless=False)
        self.tss.registerCallback(self.callback)
        rospy.spin()
        
    def callback(self,camera1_data,camera2_data):

        rgb_image_1 = None
        rgb_image_2 = None
        try:
            rgb_image_1 = self.br.imgmsg_to_cv2(camera1_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            rgb_image_2 = self.br.imgmsg_to_cv2(camera2_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        print(type(rgb_image_1))
        print(type(rgb_image_2))
        

if __name__ == '__main__':

    image_data_list = ImageReader('capture_images')
    

    