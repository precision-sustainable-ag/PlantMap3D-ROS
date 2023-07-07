#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Bool
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np
import cv2
from height_map import HeightMap

def callback(camera_data):

    bridge = CvBridge()
    # change from Image to cv using bridge 
    depth_map = bridge.imgmsg_to_cv2(camera_data.depth_map)
    boom_height = int(75)
    height_map = HeightMap(depth_map,boom_height)
    height_map_array = height_map.run() # return type ndarray
    camera_data.height_map = bridge.cv2_to_imgmsg(height_map_array)
    
    # camera_data.height_map_dims = np.shape(height_map_array)
    pub = rospy.Publisher('camera_data/height_data',numpy_msg(PM3DCameraData),queue_size=10)
    pub.publish(camera_data)
    

    

if __name__ == '__main__':

    rospy.init_node('height_map')
    while True: 
        rospy.Subscriber('camera_data',numpy_msg(PM3DCameraData),callback=callback)
        rospy.spin()