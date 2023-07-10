#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from biomass_estimator import BiomassEstimator

"""
A ROS wrapper for the biomass estimation driver.
"""

def biomass_estimator_callback(camera_data):

    """
    Callback function to listen to data published from the height map driver and compute the biomass estimation.
    """
    bridge = CvBridge()
    pub = rospy.Publisher('camera_data/biomass_estimate',numpy_msg(PM3DCameraData),queue_size=10)

    semantic_array = bridge.imgmsg_to_cv2(camera_data.segmentation_labels)
    height_array = bridge.imgmsg_to_cv2(camera_data.height_map)
    biomass_estimator_data = BiomassEstimator(semantic_array,height_array)
    biomass_estimate_res = biomass_estimator_data.run()
    biomass_estimate_res = np.array(biomass_estimate_res,dtype=np.int64)
    camera_data.biomass_estimate = biomass_estimate_res
    pub.publish(camera_data)  

if __name__ == '__main__':

    rospy.init_node("biomass_estimate")

    # numpy_msg(PM3DCameraData)
    while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_data/height_data',numpy_msg(PM3DCameraData),callback=biomass_estimator_callback)
            rospy.spin()