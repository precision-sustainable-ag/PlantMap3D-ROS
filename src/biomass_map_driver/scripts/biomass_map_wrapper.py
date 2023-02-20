#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
import numpy as np

from biomass_map import BiomassMap

def biomass_map_wrapper_callback(biomass_estimate_data):

    pass

if __name__ == '__main__':

    rospy.init_node("biomass_map_driver")
