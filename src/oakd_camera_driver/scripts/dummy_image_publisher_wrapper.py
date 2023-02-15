#!/usr/bin/env python3
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from image_publisher import TestImagePublisher

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData


if __name__ == '__main__':

    camera_data = [["mona_lisa.png","camera_1","camera_data"],["test_painting.png","camera_2","camera_data"],
    ["test_painting.png","camera_3","camera_data"],["test_painting.png","camera_4","camera_data"]]
    # camera_data = [["169.254.54.205","camera_1","camera_1_data"]]

    for cam_data in camera_data:
        
        commd = "python3 image_publisher.py {} {} {} &".format(str(cam_data[0]), str(cam_data[1]), str(cam_data[2]))
        os.system(commd)