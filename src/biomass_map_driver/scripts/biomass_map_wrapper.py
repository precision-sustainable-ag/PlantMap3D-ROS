#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
import rospkg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np
import os, sys 
from collections import Counter
from biomass_map_driver.srv import PM3DBiomassDataSaver, PM3DBiomassDataSaverRequest

rospack_testset = rospkg.RosPack()
__biomass_summary_path= rospack_testset.get_path('data_saver_driver') + '/biomass_estimation/biomass_save_summary/'

import datetime, pytz
current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))


def biomass_map_wrapper_callback(biomass_estimate_data):
    """
    This function outputs the biomass map generally in the form of PNG or JPEG
    """
    summary_directory = __biomass_summary_path #ADD the actual path here, but keep the date-based filename
    summary_filename = "biomass_data_" + str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + ".csv"
    summary_path_and_name = summary_directory + "/" + summary_filename 
    counter_biomass = Counter(biomass_estimate_data.biomass_estimate)
    counter_segmentation = Counter(biomass_estimate_data.segmentation_labels)
    print(f"Biomass estimate counter : {counter_biomass}")
    print(f"Segmentation labes counter : {counter_segmentation}")
    print(biomass_estimate_data.biomass_estimate)



if __name__ == '__main__':
    
    rospy.init_node("biomass_map_driver")
    while not rospy.is_shutdown(): 
        rospy.Subscriber('camera_data/biomass_estimate',PM3DCameraData,callback=biomass_map_wrapper_callback)
        rospy.spin()
