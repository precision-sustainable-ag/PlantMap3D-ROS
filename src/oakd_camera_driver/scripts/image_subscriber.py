#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import numpy as np
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData

"""
    This is part of the test scripts
"""

def callback(data):

    
    array_data = np.arange(27,dtype=np.int64).reshape((3,3,3))
    subscribed_rgb_data = data.rgb_data.reshape((data.rgb_dims))
    assert ((subscribed_rgb_data == array_data).all()), "Arrays are not same"
    

def listener():

    rospy.init_node("listener_node")

    rospy.Subscriber('numpy_data',numpy_msg(PM3DCameraData),callback=callback)

    rospy.spin()

if __name__ == '__main__':

    listener()