#!/usr/bin/env python3
"""
Created on Fri Apr 28 19:53:57 2023

@author: skovsen, MathewAaron
"""

import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from biomass_map_driver.srv import PM3DBiomassDataSaver, PM3DBiomassDataSaverResponse
from os import path
"""
# Test input data directly below - should be replaced with something from ros
# im_filename = "test.jpg"
# latitude = -39.1
# longitude = 71.2
# grass_pixels = 1
# grass_biomass = 2
# clover_pixels = 3
# clover_biomass = 4
# brassica_pixels = 5
# brassica_biomass = 6
# weed_pixels = 7
# weed_biomass = 8
# total_vegetation_pixels = 9
# total_biomass = 10
"""



def biomass_data_saver_callabck(biomass_estimate_data):

    if(not path.exists(biomass_estimate_data.summary_path_and_name)):
        biomass_header = ["im_filename", "latitude", "longitude", "grass_pixels", "grass_biomass", "clover_pixels", "clover_biomass", "brassica_pixels", "brassica_biomass", "weed_pixels", "weed_biomass", "total_vegetation_pixels", "total_biomass"]
        fd = open(biomass_estimate_data.summary_path_and_name,'w', newline='')
        fd.write((';'.join(map(str, biomass_header)))+'\r\n')
        fd.close()
    
    biomass_data = [biomass_estimate_data.image_name, biomass_estimate_data.latitude, 
                    biomass_estimate_data.longitude, biomass_estimate_data.grass_pixels, 
                    biomass_estimate_data.grass_biomass, biomass_estimate_data.clover_pixels, 
                    biomass_estimate_data.clover_biomass, biomass_estimate_data.brassica_pixels, 
                    biomass_estimate_data.brassica_biomass, biomass_estimate_data.weed_pixels, 
                    biomass_estimate_data.weed_biomass, biomass_estimate_data.total_vegetation_pixels, biomass_estimate_data.total_biomass]
    fd = open(biomass_estimate_data.summary_path_and_name,'a', newline='')
    fd.write((';'.join(map(str, biomass_data)))+'\r\n')
    fd.close()

    return PM3DBiomassDataSaverResponse(True) 

if __name__ == "__main__":

    rospy.init_node("biomass_csv_server")
    rospy.loginfo("Biomass Data Saver Service Requested")
    biomass_saver_srv = rospy.Service('biomass_data_saver',PM3DBiomassDataSaver,biomass_data_saver_callabck)
    rospy.spin()
