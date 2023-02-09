#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Int32
import numpy as np

"""
PM3DCameraData :
    int64[] rgb_data
    int64[] depth_map
    int64[] segmentation_labels
"""


def talker(array_data):

    rospy.init_node('talker')
    pub = rospy.Publisher('numpy_data',numpy_msg(PM3DCameraData),queue_size=1)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        
        pub.publish(array_data)
        r.sleep()


if __name__ == '__main__':

    #talker()

    array_data = np.arange(27,dtype=np.int64).reshape((3,3,3))
    print(array_data)
    depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
    segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)

    msg = PM3DCameraData()
    msg.rgb_data = array_data.flatten().tolist()
    msg.depth_map = depth_data.flatten().tolist()
    msg.segmentation_labels = segmentation_label_arr.flatten().tolist()

    talker(msg)