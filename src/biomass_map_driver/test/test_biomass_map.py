#!/usr/bin/env python3
from unittest import TestCase
import rostest 
import csv
import roslib; roslib.load_manifest('biomass_map_driver')
import rospy 
import rospkg
from collections import Counter
from biomass_map_driver.srv import PM3DBiomassDataSaver, PM3DBiomassDataSaverRequest
import numpy as np
np.random.seed(45)
import time
import os, sys 
import datetime, pytz
class TestBiomassMap(TestCase):

    def setup(self):

        rospy.init_node('biomass_saver_test')

    def get_last_csv_line(self,summary_complete_path_name:str)->list:

        with open(summary_complete_path_name,'r') as csvfile:
            contents = csvfile.read()

        last_line = contents.strip().split('\n')[-1]

        last_row = list(csv.reader([last_line]))[0]
        last_row = last_row[0].split(';')

        return last_row
    
    def test_check_append_length(self):

        rospy.wait_for_service('biomass_data_saver')
        rospack_testset = rospkg.RosPack()
        current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))
        summary_filename = "test_biomass_data_" + str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + ".csv"
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
            
            testbiomass_data = ["test.jpg", -39.1, 
                                71.2, 1, 
                                2, 3, 
                                4, 5, 
                                6, 7, 
                                8, 9, 
                                10]
            resp = data_saver(req)

            biomass_srv_output = self.get_last_csv_line(__biomass_summary_path)
            print(biomass_srv_output)
            # check if right number of elements are put in
            self.assertEqual(len(testbiomass_data),len(biomass_srv_output),"File append lenght not equal")
            

        except rospy.ServiceException as e :
            rospy.logerr("Camera location service call failed..")

if __name__ == '__main__':
    
    rostest.rosrun('biomass_map_driver','test_biomass_map.py',TestBiomassMap)
