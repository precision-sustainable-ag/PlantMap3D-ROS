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

    def __init__(self,*args):
         super(TestBiomassMap,self).__init__(*args)
         self.req = PM3DBiomassDataSaverRequest()
         self.testbiomass_data = ["test.jpg", -39.1, 
                                71.2, 1, 
                                2, 3, 
                                4, 5, 
                                6, 7, 
                                8, 9, 
                                10]

    def setup(self):
         
        rospy.wait_for_service('biomass_data_saver')
        self.rospack_testset = rospkg.RosPack()
        self.current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))
        self.summary_filename = "test_biomass_data_" + str(self.current_time.year) + "-" + str(self.current_time.month) + "-" + str(self.current_time.day) + ".csv"
        self.__biomass_summary_path= self.rospack_testset.get_path('data_saver_driver') + '/biomass_estimation/biomass_save_summary/' + str(self.summary_filename)
        self.req.summary_path_and_name = self.__biomass_summary_path
        self.req.image_name = "test.jpg"
        self.req.latitude = -39.1
        self.req.longitude = 71.2
        self.req.grass_pixels = 1
        self.req.grass_biomass = 2
        self.req.clover_pixels = 3
        self.req.clover_biomass = 4
        self.req.brassica_pixels = 5
        self.req.brassica_biomass = 6
        self.req.weed_pixels = 7
        self.req.weed_biomass = 8
        self.req.total_vegetation_pixels = 9
        self.req.total_biomass = 10
        

    def get_last_csv_line(self,summary_complete_path_name:str)->list:

        with open(summary_complete_path_name,'r') as csvfile:
            contents = csvfile.read()

        last_line = contents.strip().split('\n')[-1]

        last_row = list(csv.reader([last_line]))[0]
        last_row = last_row[0].split(';')

        return last_row
    
    def test_check_append_length(self):
        
        try :

            data_saver = rospy.ServiceProxy('biomass_data_saver',PM3DBiomassDataSaver)
  
            resp = data_saver(self.req)

            biomass_srv_output = self.get_last_csv_line(self.__biomass_summary_path)
            print(biomass_srv_output)
            # check if right number of elements are put in
            self.assertEqual(len(self.testbiomass_data),len(biomass_srv_output),"File append lenght not equal")
        
        except rospy.ServiceException as e :
            rospy.logerr("Camera location service call failed..")

    # def test_element_type(self):

    #         for i in range(len)
if __name__ == '__main__':
    rospy.init_node('biomass_saver_test')
    rostest.rosrun('biomass_map_driver','test_biomass_map.py',TestBiomassMap)
