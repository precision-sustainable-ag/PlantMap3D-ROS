#!/usr/bin/env python3

from unittest import TestCase
import numpy as np
import os, sys

import rostest 
import rospkg
import roslib; roslib.load_manifest('height_map_driver')
import rospy
from rospy.numpy_msg import numpy_msg
__path = rospkg.RosPack()
__heightpath = __path.get_path('height_map_driver') + '/scripts/'

sys.path.append(__heightpath)
from height_map import HeightMap

from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import String
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np
np.random.seed(45)

class TestHeightMap(TestCase):

    def __init__(self,*args):
        super(TestHeightMap,self).__init__(*args)

        self.bridge = CvBridge()
        __path = rospkg.RosPack()
        self.__test_rgbdatapath = __path.get_path('oakd_camera_driver') + '/test/rgb/1674584672.191502.jpg'
        self.__test_depthdatapath = __path.get_path('oakd_camera_driver') + '/test/depth/1674584672.191502.png'
        self.__test_segmentationpath = __path.get_path('oakd_camera_driver') + '/test/segmentation/1674584672.191502.jpg'


    def test_run(self):
        test_input_1 = np.ones([2, 3])
        test_input_2 = 1
        hm = HeightMap(depth_map=test_input_1, boom_height=test_input_2)
        hm_results = hm.run()
        self.assertEqual(hm_results[0][0], 2.0)

if __name__ == '__main__':    
    rostest.rosrun('height_map_driver','test_height_map.py',TestHeightMap)