#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np

from biomass_map import BiomassMap



rospack = rospkg.RosPack()
__path = rospack.get_path('configs') + '/config/species_list.json'

def biomass_map_wrapper_callback(biomass_estimate_data):
    """
    This function outputs the biomass map generally in the form of PNG or JPEG
    """
    global __path
    biomass_map_object = BiomassMap(biomass_estimate_data.biomass_estimate,[biomass_estimate_data.gps_data.latitude,biomass_estimate_data.gps_data.longitude],__path)
    biomass_map = biomass_map_object.run()
    print(f"Biomass map is : {biomass_map}")

if __name__ == '__main__':
    
    rospy.init_node("biomass_map_driver")
    while not rospy.is_shutdown(): 
        rospy.Subscriber('camera_data/biomass_estimate',numpy_msg(PM3DCameraData),callback=biomass_map_wrapper_callback)
        rospy.spin()
