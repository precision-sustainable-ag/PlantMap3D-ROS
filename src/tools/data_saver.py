#!/usr/bin/env python3

import os
import sys
import numpy as np
import cv2 

@staticmethod
def check_dir_exists(image_save_path:str):

    if not os.path.exists(image_save_path):
        # os.makedirs(image_save_path)
        print("Directory does not exist..")
        return False
    else:
        print("Directory already exists..")
        return True

def data_saver_module(image_data:np.ndarray, image_save_path:str):





    return 0