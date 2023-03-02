#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool


def get_gps_data(gps_string:str):

    return gps_string 


if __name__ == '__main__':

    rospy.init_node("camera_trigger")

    camera_trigger = True
    rate = rospy.Rate(1)
    pub = rospy.Publisher("camera_trigger",Bool,queue_size=1)
    # gps_string = "$GNRMC,175800.00,A,3546.48455,N,07841.05018,W,0.212,,010223,,,A,V*04"

    while not rospy.is_shutdown():
        
        rospy.loginfo(f"triggering camera : {camera_trigger}")
        pub.publish(camera_trigger)
        rate.sleep()

    