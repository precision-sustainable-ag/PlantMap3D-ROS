#!/usr/bin/env python3

import rospy
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest

if __name__ == '__main__':

    rospy.init_node("cam_location_client")
    rospy.wait_for_service("camera_location_service")


    try:
        cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        req = PM3DCameraLocationRequest()
        req.gpscoords = [1.0,50.0]
        req.gpsheading = 90.0
        req.cameraid = 4
        
        resp = cam_location(req)
        
        rospy.loginfo(f"Received Camera 2 Location : {resp.newgpscoords}")
    except rospy.ServiceException as e:
        rospy.logerr("Camera Location Service Call Failed..")