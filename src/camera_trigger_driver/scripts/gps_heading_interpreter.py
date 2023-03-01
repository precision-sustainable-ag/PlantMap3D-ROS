#!/usr/bin/env python3

import rospy 

from std_msgs.msg import NavSatFix, NavSatStatus, TimeReference 
from std_msgs import String
import math 


class GPSHeadingInterpreter():


    def __init__(self):

        rospy.init_node("gps_heading_interpreter")
        self.messages = []
        self.sub = rospy.Subscriber('/fix',NavSatFix,callback=self.callback)
        self.pub = rospy.Publisher('fix/gps_heading',String,queue_size=1)


    def callback(self,gps_fix_data):

        self.messages.append(gps_fix_data)

        if len(self.messages) == 5:

            # do something
            # debugging
            
            gps_reading_1 = self.messages[0].data
            gps_reading_2 = self.messages[1].data
            self.messages.clear()

    def publish_gps_data(self):

        pass 

