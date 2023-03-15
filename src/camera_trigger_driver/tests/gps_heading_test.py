#!/usr/bin/env python3

import unittest
import rostest
import os 
import time
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('camera_trigger_driver')
import rospy

class TestGPSHeading(unittest.TestCase):

    pass