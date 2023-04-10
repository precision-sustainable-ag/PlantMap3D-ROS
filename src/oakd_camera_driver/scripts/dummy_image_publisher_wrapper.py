#!/usr/bin/env python3
import os 
import subprocess
import yaml
import sys

__path = os.getcwd() # + '/config/image_config.yaml'
print( __path)
try :

    if not os.path.exists(__path):
        raise FileNotFoundError(f"File not found at path : {__path}")
    else:
        # rospy.loginfo("Getting camera info...")
        with open(__path,'r+') as f:
            node_data = yaml.safe_load(f)

        node_names = list(node_data.keys())
        # rospy.loginfo(f"Initializing camera node names as : {node_names}")

except FileNotFoundError as e:
    print(e)
import rospy
import roslib; roslib.load_manifest('oakd_camera_driver')

from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import rospkg

# os.chdir(os.path.dirname(os.path.abspath(__file__)))

# """
#     This is part of the test scripts
# """
# if __name__ == '__main__':

#     rospy.init_node("camera_wrapper")
#     # temporary camera id
#     camera_data = 0
#     for node_name in node_names :
    
#         camera_data.append(["python3","image_publisher.py",node_name,camera_data])

#     processes = []
#     for cam_data in camera_data:
        
#         process = subprocess.Popen(cam_data)
#         processes.append(process)
    
#     rospy.spin()

    