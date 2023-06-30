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
from check_camera import check_camera_connection

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
    # camera_data = [["python3","oak_collection.py","169.254.54.205","camera_1","1"]]
    # camera_data = [["python3","oak_collection.py","169.254.54.200","camera_2","2"]]
    camera_data = []
    
    for node_name in node_names :
        cameraid = node_data[node_name]['camera_id']
        camera_ip = node_data[node_name]['ip']
        process_file = "nn_iso.py"
        if check_camera_connection(camera_ip):
            camera_data.append(["python3",process_file,camera_ip,node_name,str(cameraid)])
    
    print(camera_data)

    processes = []
    for cam_data in camera_data:
        process = subprocess.Popen(cam_data)
        processes.append(process)

    rospy.spin()