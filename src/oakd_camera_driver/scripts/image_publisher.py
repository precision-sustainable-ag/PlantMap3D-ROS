#!/usr/bin/env python3

import numpy as np
import cv2
import os
import sys 



def get_camera_data(rgb_image:np.ndarray,depth_map:np.ndarray,segmentation_labels:np.ndarray):


    return rgb_image, depth_map, segmentation_labels


if __name__ == '__main__':

    rgb_image = cv2.imread("~/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/oakd_camera_driver/scripts/mona_lisa.png")
    depth_map = np.random.randint(0,2,(128,128,3),dtype=int)
    segmentation_labels = np.random.randint(0,4,(128,128,3),dtype=int)
    



