#!/usr/bin/env python3
"""
@author: MathewAaron
"""

import rospy
# from flask import Flask
import os 

if __name__ == '__main__':

    rospy.init_node("flask_app_node")
    os.system("roslaunch pm3d_system_config system_start.launch")

    