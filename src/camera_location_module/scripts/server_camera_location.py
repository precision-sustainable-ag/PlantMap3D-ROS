#!/usr/bin/env python3

import rospy 
from camera_location_module import PM3DCameraLocation
from camera_location import find_camera_gps_coordinates
class CameraLocationInterpreter():
    """
    This class is a ROS server that takes in Camera Coords (lat and lon),
    GPS Heading, Camera ID and Camera Config path location and returns new camera location
    """
    def __init__(self):

        rospy.init_node("camera_location_interpreter")

    def camera_location_callback(self,data):

        rospy.loginfo(f"Printing GPS coords from camera location srv : {data.gpscoords}") 
        return 0
    
    def camera_location_server(self):

        rospy.loginfo(f"Camera Location service requested ")
        cam_location_service = rospy.Service('camera_location_service',PM3DCameraLocation,self.camera_location_callback)
        rospy.spin()