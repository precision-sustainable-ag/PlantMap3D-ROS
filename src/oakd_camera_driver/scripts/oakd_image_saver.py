#!/usr/bin/env python3

import os
import rospy
import sys
parent_dir = os.path.abspath("/home/plantmap3d/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/data_saver_driver/")
import rospkg
rospack = rospkg.RosPack()
import yaml
import subprocess
os.chdir(parent_dir)
__path = rospack.get_path('configs') + '/config/image_config.yaml'
from check_camera import check_camera_connection


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

processes = []
# node_data, node_names = get_camera_nodes()
for node_name in node_names:
    camera_ip = node_data[node_name]['ip']
    camera_id = node_data[node_name]['camera_id']
    if check_camera_connection(camera_ip):
        new_path = parent_dir + f"/images_{camera_id}"
        if not os.path.exists(new_path):
            os.makedirs(new_path)
        command = ["rosrun", "image_view", "image_saver", f"image:=/camera{camera_id}_image"]
        # command = ["rosrun", "image_view", "image_view", f"image:=/camera{camera_id}_image"]
        processes += [subprocess.Popen(command, cwd=new_path)]
for p in processes:
    p.wait()


if __name__ == "__main__":

    rospy.init_node('image_saver')

    while not rospy.is_shutdown():
        rospy.spin()