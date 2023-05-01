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
from collections import Counter
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
    camera_data.biomass_estimate = biomass_estimate_res
    counter_biomass = Counter(camera_data.biomass_estimate)
    counter_segmentation = Counter(camera_data.segmentation_labels)
    print(f"Biomass estimate counter : {counter_biomass}")
    print(f"Segmentation labes counter : {counter_segmentation}")
    print(camera_data.biomass_estimate)
    print("----------------------------")
    rospy.loginfo("Publishing biomass estimates")
    pub = rospy.Publisher('camera_data/biomass_estimate',numpy_msg(PM3DCameraData),queue_size=10)
    pub.publish(camera_data)  

if __name__ == '__main__':

    rospy.init_node("biomass_estimate")

    # numpy_msg(PM3DCameraData)
    while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_data/height_data',numpy_msg(PM3DCameraData),callback=biomass_estimator_callback)
            rospy.spin()