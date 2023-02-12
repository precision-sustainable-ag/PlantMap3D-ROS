#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Bool
import numpy as np
import cv2
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))
"""
PM3DCameraData :
    int64[] rgb_data
    int64[] depth_map
    int64[] segmentation_labels
    int64[] rgb_dims
    int64[] depth_map_dims
    int64[] segmentation_label_dims
    int64[] height_map
"""


# def image_publisher(array_data):

#     rospy.init_node('talker')
#     pub = rospy.Publisher('numpy_data',numpy_msg(PM3DCameraData),queue_size=1)

#     r = rospy.Rate(10)

#     while not rospy.is_shutdown():

        
#         pub.publish(array_data)
#         r.sleep()

image_data = cv2.imread("mona_lisa.png")

def callback(data):

    global image_data    
    array_data = cv2.resize(image_data,(1024,1024),cv2.INTER_CUBIC)
    depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
    segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)
    msg = PM3DCameraData()
    msg.rgb_data = array_data.flatten().tolist()
    msg.depth_map = depth_data.flatten().tolist()
    msg.segmentation_labels = segmentation_label_arr.flatten().tolist()

    msg.rgb_dims = array_data.shape
    msg.depth_map_dims = depth_data.shape
    msg.segmentation_label_dims = segmentation_label_arr.shape

    pub = rospy.Publisher('camera_data',numpy_msg(PM3DCameraData),queue_size=1)

    if data.data == True:
        """
        When camera trigger is true, publish camera image data
        """
        rospy.loginfo("Publishing Camera Data")
        pub.publish(msg)
    else:
        rospy.loginfo("Not Publishing")



if __name__ == '__main__':

    #talker()
    rospy.init_node('image_publisher')
    while True: 
            rospy.Subscriber('camera_trigger',Bool,callback=callback)
            rospy.spin()

    