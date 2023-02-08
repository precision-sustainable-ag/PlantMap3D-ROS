#!/usr/bin/env python3

import numpy as np
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
"""
PM3DCameraData :
    int64[] rgb_data
    int64[] depth_map
    int64[] segmentation_labels
"""
from std_msgs.msg import Int32
import numpy as np


# def get_camera_data(rgb_image:np.ndarray,depth_map:np.ndarray,segmentation_labels:np.ndarray):


#     return rgb_image, depth_map, segmentation_labels


# if __name__ == '__main__':

#     rgb_image = cv2.imread("~/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/oakd_camera_driver/scripts/mona_lisa.png")
#     depth_map = np.random.randint(0,30,(128,128,3),dtype=int)
#     segmentation_labels = np.random.randint(0,4,(128,128,3),dtype=int)
    



def talker(array_data):

    rospy.init_node('talker')
    pub = rospy.Publisher('numpy_data',numpy_msg(PM3DCameraData),queue_size=1)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        
        pub.publish(array_data)
        r.sleep()


if __name__ == '__main__':

    #talker()
    array_data = np.array(([1,2,3,4],[5,6,7,8]),dtype=np.int64)
    depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
    segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)

    msg = PM3DCameraData()
    msg.rgb_data = array_data.flatten().tolist()
    msg.depth_map = depth_data.flatten().tolist()
    msg.segmentation_labels = segmentation_label_arr.flatten().tolist()

    talker(msg)