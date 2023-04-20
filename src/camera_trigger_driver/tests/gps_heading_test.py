#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import unittest
import rostest
import os 
import time
import math
os.chdir(os.path.dirname(os.path.abspath(__file__)))
from camera_trigger_driver.srv import  PM3DGPSHeading, PM3DGPSHeadingRequest

import roslib; roslib.load_manifest('camera_trigger_driver')
import rospy

class TestGPSHeading(unittest.TestCase):

    def setup(self):
        
        rospy.init_node('gps_test_client')

    def test_gps_msg_type(self):

        northing = 1200.0
        lnorthing = 1210.0
        easting = 930.0
        leasting = 349.0
        
        diff_easting = easting - leasting
        diff_northing = northing - lnorthing

        test_gps_heading = math.atan2(diff_easting,diff_northing)
        test_gps_heading = math.degrees(test_gps_heading)

        gps_data = PM3DGPSHeadingRequest()
        gps_data.northing = northing
        gps_data.lnorthing = lnorthing
        gps_data.easting = easting
        gps_data.leasting = leasting

        self.assertEqual(northing,gps_data.northing)
        self.assertEqual(lnorthing,gps_data.lnorthing)
        self.assertEqual(easting,gps_data.easting)
        self.assertEqual(leasting,gps_data.leasting)
    
    def test_gps_heading(self):

        
        northing = 1200.0
        lnorthing = 1210.0
        easting = 930.0
        leasting = 349.0
        
        diff_easting = easting - leasting
        diff_northing = northing - lnorthing

        test_gps_heading = math.atan2(diff_easting,diff_northing)
        test_gps_heading = math.degrees(test_gps_heading)

        gps_data = PM3DGPSHeadingRequest()
        gps_data.northing = northing
        gps_data.lnorthing = lnorthing
        gps_data.easting = easting
        gps_data.leasting = leasting
        
        srv_gps_heading = rospy.ServiceProxy('gps_heading_service',PM3DGPSHeading)

        response = srv_gps_heading(gps_data)
        self.assertAlmostEqual(test_gps_heading,response.gps_heading,places=3)
        


if __name__ == '__main__':
    
    rostest.rosrun('camera_trigger_driver','gps_heading_test',TestGPSHeading)

    