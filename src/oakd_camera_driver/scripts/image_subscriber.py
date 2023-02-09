#!/usr/bin/env python3

import numpy as np
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData

def callback(data):

    # rospy.loginfo(f"Got depth data :{data.depth_map}")
    # getting image data as list and converting back to numpy arrays
    # print("depth data is : ", data.depth_map.reshape(128,128,3))
    # array_data = np.arange(27,dtype=np.int64).reshape((3,3,3))
    array_data = np.ones((3,3,3))
    subscribed_rgb_data = data.rgb_data.reshape((3,3,3))
    assert ((subscribed_rgb_data == array_data).all()), "Arrays are not same"
    

def listener():

    rospy.init_node("listener_node")

    rospy.Subscriber('numpy_data',numpy_msg(PM3DCameraData),callback=callback)

    rospy.spin()

if __name__ == '__main__':

    listener()