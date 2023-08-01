#!/usr/bin/env python3

import rospy 
from camera_trigger_driver.srv import  PM3DGPSHeading
import math 

def deg_to_rads(gps_coord):
     
     return math.radians(gps_coord)

def compute_heading(prev_lat, prev_lon,curr_lat,curr_lon):
     """
     This function computes GPS heading based on 
     """
     curr_lat_rad = deg_to_rads(curr_lat)
     curr_lon_rad = deg_to_rads(curr_lon)
     prev_lat_rad = deg_to_rads(prev_lat)
     prev_lon_rad = deg_to_rads(prev_lon)


     diff_lon = curr_lon_rad - prev_lon_rad
     X = math.cos(curr_lat_rad) * math.sin(diff_lon)
     Y = math.cos(prev_lat_rad) * math.sin(curr_lat_rad) - math.sin(prev_lat_rad) * math.cos(curr_lat_rad) * math.cos(diff_lon)


     gps_heading_rad = math.atan2(X,Y)
     gps_heading_deg = math.degrees(gps_heading_rad)

     # Ensuring heading_deg is within [0, 360)
     gps_heading_deg = (gps_heading_deg + 360) % 360 
     return float(gps_heading_deg)

class GPSHeadingInterpreter():

    """
    This class computes the GPS heading in degrees
    Return type float
    """
 
    def __init__(self):

        rospy.init_node("gps_heading_interpreter")
        
    def gps_heading_callback(self,gps_msg):
        # compute_heading(lat1,lon1,lat2,lon2)
        gps_heading = compute_heading(float(gps_msg.prev_lat),float(gps_msg.prev_lon),
                                      float(gps_msg.current_lat),float(gps_msg.current_lon))
        return gps_heading

    def gps_heading_server(self):

        rospy.loginfo(f"GPS Heading service requested ")
        gps_heading_service = rospy.Service('gps_heading_service',PM3DGPSHeading,self.gps_heading_callback)

        rospy.spin()  

if __name__ == "__main__":

    gps_heading_obj = GPSHeadingInterpreter()