#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import os 
import subprocess
import yaml
import sys
import rospy
import roslib; roslib.load_manifest('oakd_camera_driver')

from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import rospkg

rospy.init_node("camera_wrapper")

rospack = rospkg.RosPack()
__path = rospack.get_path('configs') + '/config/image_config.yaml'

try :

    if not os.path.exists(__path):
        raise FileNotFoundError(f"File not found at path : {__path}")
    else:
        rospy.loginfo("Getting camera info...")
        with open(__path,'r+') as f:
            node_data = yaml.safe_load(f)

        node_names = list(node_data.keys())
        rospy.loginfo(f"Initializing camera node names as : {node_names}")

except FileNotFoundError as e:
    print(e)

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# """
#     This is part of the test scripts
# """
if __name__ == '__main__':

    camera_data = []
    
    for node_name in node_names :
        cameraid = node_data[node_name]['camera_id']
        camera_data.append(["python3","image_publisher.py",node_name,str(cameraid)])

    processes = []
    for cam_data in camera_data:
        
        process = subprocess.Popen(cam_data)
        processes.append(process)
    
    rospy.spin()

    