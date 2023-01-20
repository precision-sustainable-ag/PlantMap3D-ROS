#!/usr/bin/env python3

"""
    Currently this driver will publish mock data 
"""
import rospy 
from std_msgs.msg import Bool 
import time 

if __name__ == '__main__':

    try :
        rospy.init_node('gps_driver')

        rospy.loginfo('Mocking GPS reading every 5 secs...')
        bool_publish = rospy.Publisher('/gps_data',Bool,queue_size=1)
        while True:
            rospy.loginfo('Publishing "True" as data')
            bool_publish.publish(True)
            time.sleep(5)

    except rospy.ROSInterruptException:
        pass