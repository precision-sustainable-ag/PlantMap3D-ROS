#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Bool
import numpy as np

from height_map import HeightMap

def callback(camera_data):

    depth_map = camera_data.depth_map.reshape((camera_data.depth_map_dims))
    boom_height = float(75)
    height_map = HeightMap(depth_map,boom_height)
    height_map_array = height_map.run() # return type ndarray
    camera_data.height_map = height_map_array.flatten()
    
    camera_data.height_map_dims = np.array(height_map_array.shape)

    rospy.loginfo("Publishing Camera Data with height array")
    print(camera_data)
    print("--------------------------")
    pub = rospy.Publisher('camera_data/height_data',numpy_msg(PM3DCameraData),queue_size=10)
    pub.publish(camera_data)
    

    

if __name__ == '__main__':

    rospy.init_node('height_map')
    while True: 
        rospy.Subscriber('camera_data',numpy_msg(PM3DCameraData),callback=callback)
        rospy.spin()