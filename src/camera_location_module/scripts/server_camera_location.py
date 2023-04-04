#!/usr/bin/env python3

import rospy 
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationResponse
from camera_location import find_camera_gps_coordinates

class CameraLocationInterpreter():
    """
    This class is a ROS server that takes in Camera Coords (lat and lon),
    GPS Heading, Camera ID and Camera Config path location and returns new camera location
    """
    def __init__(self):

        rospy.init_node("camera_location_interpreter")
        self.camera_location_server()

    def camera_location_callback(self,data):

        rospy.loginfo(f"Printing GPS coords from camera location srv : {data.gpscoords}") 
        camera_coords = find_camera_gps_coordinates(data.gpscoords,data.gpsheading,data.cameraid)
        return PM3DCameraLocationResponse(camera_coords)
    
    def camera_location_server(self):

        rospy.loginfo(f"Camera Location service requested ")
        cam_location_service = rospy.Service('camera_location_service',PM3DCameraLocation,self.camera_location_callback)
        rospy.spin()

if __name__ == "__main__":

        camera_location_obj = CameraLocationInterpreter()
