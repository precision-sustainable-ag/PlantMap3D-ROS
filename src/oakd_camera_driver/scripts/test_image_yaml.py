#!/usr/bin/env python3
import os 
import cv2
import glob
import yaml 

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
import rospkg

rospy.init_node("test_set_publisher")
rospack_testset = rospkg.RosPack()
__path = rospack_testset.get_path('oakd_camera_driver') + '/tests/test_set.yaml'
__rgbpath = rospack_testset.get_path('oakd_camera_driver') + '/tests/rgb/'

image_files = glob.glob(os.path.join(__rgbpath,'*.jpg'))
image_list = []
for image_file in image_files:

    img = cv2.imread(image_file)
    image_list.append(img)

img1 = cv2.resize(image_list[0], dsize=(1024,1024))
print(img1.shape)
# cv2.imshow('Image',image_list[1])
# cv2.waitKey(0)