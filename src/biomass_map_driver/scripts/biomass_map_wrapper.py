#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np
import os, sys 
from biomass_map import BiomassMap



rospack_species = rospkg.RosPack()
__path = rospack_species.get_path('configs') + '/config/species_list.json'

rospack_datasave = rospkg.RosPack()
__biomass_data_save_path = rospack_datasave.get_path('data_saver_driver') + '/biomass_estimation/'


def get_image_name(cameraid, timestamp):

    image_name = "image_"+str(cameraid)+"_"+str(timestamp)+".png"
    return image_name


def biomass_map_wrapper_callback(biomass_estimate_data):
    """
    This function outputs the biomass map generally in the form of PNG or JPEG
    """
    global __path , __biomass_data_save_path

    image_name = get_image_name(biomass_estimate_data.camera_id,biomass_estimate_data.header.stamp)
    image_save_name = __biomass_data_save_path + image_name
    biomass_map_object = BiomassMap(biomass_estimate_data.biomass_estimate,[biomass_estimate_data.gps_data.latitude,biomass_estimate_data.gps_data.longitude],__path)
    biomass_map = biomass_map_object.run()
    print(f"Biomass map is : {biomass_map}")
    biomass_map.save(image_save_name)
    # biomass_map.show(f"{image_name}")

if __name__ == '__main__':
    
    rospy.init_node("biomass_map_driver")
    while not rospy.is_shutdown(): 
        rospy.Subscriber('camera_data/biomass_estimate',numpy_msg(PM3DCameraData),callback=biomass_map_wrapper_callback)
        rospy.spin()
