#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
import rospkg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np
import os, sys 
from collections import Counter
from biomass_map_driver.srv import PM3DBiomassDataSaver, PM3DBiomassDataSaverRequest
import datetime, pytz



def biomass_map_wrapper_callback(biomass_estimate_data):
    """
    This function outputs the biomass estimation outputs in a csv file located (default) at /data_saver_driver/biomass_estimation/biomass_save_summary/
    """
    current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))
    rospack_testset = rospkg.RosPack()
    #ADD the actual path here, but keep the date-based filename
    __biomass_summary_path= rospack_testset.get_path('data_saver_driver') + '/biomass_estimation/biomass_save_summary/'
    __summary_filename = "biomass_data_" + str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + ".csv"
    summary_complete_path_name = __biomass_summary_path + __summary_filename
    image_name = "image_" + str(biomass_estimate_data.camera_name) + "_" +  str(current_time.year) + "_" + str(current_time.month) + "_" + str(current_time.day)+".jpg" 
    counter_biomass = Counter(biomass_estimate_data.biomass_estimate)
    counter_segmentation = Counter(biomass_estimate_data.segmentation_labels)
    # print(biomass_estimate_data)
   

    # try :

    #     data_saver = rospy.ServiceProxy('biomass_data_saver',PM3DBiomassDataSaver)

    #     req = PM3DBiomassDataSaverRequest()
    #     req.summary_path_and_name = summary_complete_path_name
    #     req.image_name = image_name
    #     req.latitude = float(biomass_estimate_data.latitude)
    #     req.longitude = float(biomass_estimate_data.longitude)
    #     req.grass_pixels = 1
    #     req.grass_biomass = 2
    #     req.clover_pixels = 3
    #     req.clover_biomass = 4
    #     req.brassica_pixels = 5
    #     req.brassica_biomass = 6
    #     req.weed_pixels = 7
    #     req.weed_biomass = 8
    #     req.total_vegetation_pixels = 9
    #     req.total_biomass = 10

    #     resp = data_saver(req)

    #     rospy.loginfo(f"Saving {req} data to biomass summary...")
    #     rospy.loginfo(f"Response from server : {resp}")

    # except rospy.ServiceException as e :
    #     rospy.logerr("Camera location service call failed..")



if __name__ == '__main__':
    
    rospy.init_node("biomass_map")
    while not rospy.is_shutdown(): 
        rospy.wait_for_service('biomass_data_saver')
        rospy.Subscriber('camera_data/biomass_estimate',numpy_msg(PM3DCameraData),callback=biomass_map_wrapper_callback)
        rospy.spin()
