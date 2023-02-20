#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np

from biomass_map import BiomassMap

def biomass_map_wrapper_callback(biomass_estimate_data):

    biomass_map_object = BiomassMap(biomass_estimate_data.biomass_estimate,[1.0,1.0])
    biomass_map = biomass_map_object.run()
    print(f"Biomass map is : {biomass_map}")

if __name__ == '__main__':

    
    rospy.init_node("biomass_map_driver")

    
    while not rospy.is_shutdown(): 
        rospy.Subscriber('camera_data/height_data',numpy_msg(PM3DCameraData),callback=biomass_map_wrapper_callback)
        rospy.spin()

    
