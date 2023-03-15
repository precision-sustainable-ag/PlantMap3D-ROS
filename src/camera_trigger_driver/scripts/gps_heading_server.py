#!/usr/bin/env python3

import rospy 

from camera_trigger_driver.srv import  PM3DGPSHeading

import math 


class GPSHeadingInterpreter():


    def __init__(self):

        rospy.init_node("gps_heading_interpreter")
        self.gps_heading_server()
        

    def compute_heading(self,diff_northing, diff_easting):

        gps_heading = math.atan2(diff_easting,diff_northing)
        gps_heading = math.degrees(gps_heading)

        return gps_heading
    
    def gps_heading_callback(self,gps_fix_data):

        gps_msg = PM3DGPSHeading()
        diff_northing = gps_msg.NORTHING - gps_msg.L_NORTHING
        diff_easting = gps_msg.EASTING - gps_msg.L_EASTING
        
        gps_heading = self.compute_heading(diff_northing,diff_easting)

        rospy.loginfo(f"GPS heading computed is : {gps_heading}")
        return gps_heading

    def gps_heading_server(self):

        rospy.loginfo(f"GPS Heading service requested ")
        gps_heading_service = rospy.Service('gps_heading_service',PM3DGPSHeading,self.gps_heading_callback)

        rospy.spin() 

if __name__ == "__main__":

    gps_heading_obj = GPSHeadingInterpreter()