#!/usr/bin/env python3
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from image_publisher import TestImagePublisher

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import subprocess

if __name__ == '__main__':

    camera_data = [["python3","image_publisher.py","camera_1","camera_data"],["python3","image_publisher.py","camera_2","camera_data"],
    ["python3","image_publisher.py","camera_3","camera_data"],["python3","image_publisher.py","camera_4","camera_data"],
    ["python3","image_publisher.py","camera_5","camera_data"],["python3","image_publisher.py","camera_6","camera_data"]]

    processes = []
    for cam_data in camera_data:
        
        process = subprocess.Popen(cam_data)
        processes.append(process)

    # for process in processes:

    #     #process.wait()