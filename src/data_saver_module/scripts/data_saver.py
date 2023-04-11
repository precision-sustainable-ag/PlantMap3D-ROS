#!/usr/bin/env python3
import rospy 
from PIL import Image


def data_saver_module(image_data,cameraid,image_save_path):

    """
        This function takes in a PILLOW image returned from the biomass map wrapper and saves it in a desired path
    """

    image_data.save(image_save_path)

    return 0