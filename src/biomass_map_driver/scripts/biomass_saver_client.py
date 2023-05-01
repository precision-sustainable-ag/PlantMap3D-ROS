#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver'); roslib.load_manifest('biomass_map_driver')
import rospy
import rospkg
from oakd_camera_driver.msg import PM3DCameraData
from collections import Counter
from biomass_map_driver.srv import PM3DBiomassDataSaver, PM3DBiomassDataSaverRequest

import datetime, pytz


if __name__ == "__main__":

    rospy.init_node("biomass_saver_client")

    rospy.wait_for_service('biomass_data_saver')
    rospack_testset = rospkg.RosPack()
    current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))
    summary_filename = "biomass_data_" + str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + ".csv"
    __biomass_summary_path= rospack_testset.get_path('data_saver_driver') + '/biomass_estimation/biomass_save_summary/' + str(summary_filename)

    try :

        data_saver = rospy.ServiceProxy('biomass_data_saver',PM3DBiomassDataSaver)

        req = PM3DBiomassDataSaverRequest()
        req.summary_path_and_name = __biomass_summary_path
        req.image_name = "test.jpg"
        req.latitude = -39.1
        req.longitude = 71.2
        req.grass_pixels = 1
        req.grass_biomass = 2
        req.clover_pixels = 3
        req.clover_biomass = 4
        req.brassica_pixels = 5
        req.brassica_biomass = 6
        req.weed_pixels = 7
        req.weed_biomass = 8
        req.total_vegetation_pixels = 9
        req.total_biomass = 10

        resp = data_saver(req)

        rospy.loginfo(f"Saving {req} data to biomass summary...")
        rospy.loginfo(f"Response from server : {resp}")

    except rospy.ServiceException as e :
        rospy.logerr("Camera location service call failed..")