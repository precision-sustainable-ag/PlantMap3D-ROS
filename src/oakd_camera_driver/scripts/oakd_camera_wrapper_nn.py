#!/usr/bin/env python3
"""
@author: MathewAaron
"""
"""
    Code for python wrapper
"""
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy 
import subprocess
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import rospkg
import yaml
from IPython import embed

rospack = rospkg.RosPack()
__path = rospack.get_path('configs') + '/config/image_config.yaml'


try:
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

if __name__ == '__main__':
    """
        A camera wrapper for the OakD S2 PoE cameras. 
        This method will parallely start the defined cameras.
    """
    rospy.init_node("camera_wrapper")
    # camera_data = [["python3","nn_iso.py","169.254.54.205","camera_1","1"], ["python3","nn_iso.py","169.254.54.200","camera_2","2"],["python3","nn_iso.py","169.254.54.190","camera_3","3"],["python3","nn_iso.py","169.254.54.196","camera_4","4"]]
    camera_data = [["python3","oak_collection.py","169.254.54.205","camera_1","1"]]
    # camera_data = [["python3","oak_collection.py","169.254.54.200","camera_2","2"]]

    processes = []
    for cam_data in camera_data:
        
        process = subprocess.Popen(cam_data)
        processes.append(process)

    rospy.spin()