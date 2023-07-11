#!/usr/bin/env python3

"""
authors : @danielginn , @MathewAaron 

NOTE : TO RUN THE TEST 

    PlantMap3D/
        catkin_make run_tests_biomass_estimator_driver

"""
import rostest 
import rospkg
import roslib; roslib.load_manifest('biomass_estimator_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from unittest import TestCase
import numpy as np
np.random.seed(45)

__path = rospkg.RosPack()
__biomasspath = __path.get_path('biomass_estimator_driver') + '/scripts/'
import os, sys 
sys.path.append(__biomasspath)
from biomass_estimator import BiomassEstimator


class TestBiomassEstimator(TestCase):

    def __init__(self,*args):
        super(TestBiomassEstimator,self).__init__(*args)
        self.test_input_1 = np.ones([2, 3])
        self.test_input_2 = np.ones([2, 3])


    def test_should_output_a_numpy_array(self):
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(type(biomass_results), np.ndarray)

    def test_should_sum_to_five_ones(self):
        self.test_input_1 = np.ones([2, 3])
        self.test_input_1[0][0] = 0
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(biomass_results[1], 5)

    def test_should_sum_to_two_twos(self):
        self.test_input_1 = np.ones([2, 3])
        self.test_input_1[0][1] = 2
        self.test_input_1[1][0] = 2
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(biomass_results[2], 2)

    def test_should_make_output_array_of_length_3(self):
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        self.assertEqual(bio_est.num_categories, 3)

if __name__ == '__main__':
    rostest.rosrun('biomass_estimator_driver','test_biomass_estimator.py',TestBiomassEstimator)