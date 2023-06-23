import rospkg
import yaml
import rospy
import os
rospack = rospkg.RosPack()
__path = rospack.get_path('configs') + '/config/image_config.yaml'

def get_camera_nodes():
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
    return node_data, node_names

# node_data, node_names = get_camera_nodes()
# print(node_data, node_names)
