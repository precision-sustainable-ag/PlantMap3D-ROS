#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np

from biomass_estimator import BiomassEstimator

"""
A ROS wrapper for the biomass estimation driver.
"""

def biomass_estimator_callback(camera_data):

    """
    Callback function to listen to data published from the height map driver and compute the biomass estimation.
    """

    semantic_array = camera_data.segmentation_labels.reshape(camera_data.segmentation_label_dims)
    height_array = camera_data.height_map.reshape(camera_data.height_map_dims)
    biomass_estimator_data = BiomassEstimator(semantic_array,height_array)
    biomass_estimate_res = biomass_estimator_data.run()
    rospy.loginfo("Publishing biomass estimates")
    print(biomass_estimate_res)

if __name__ == '__main__':

    rospy.init_node("biomass_estimate")

    while True: 
            rospy.Subscriber('camera_data/height_data',numpy_msg(PM3DCameraData),callback=biomass_estimator_callback)
            rospy.spin()