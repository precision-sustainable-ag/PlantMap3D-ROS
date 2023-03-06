#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Log
import threading

def get_gps_data():
    """
        This function is a psuedo function that publishes a camera trigger based on regular intervals, 
        this trigger is used to capture and publish camera data

        [This file is just for integration testing]
    """
    camera_trigger = True
    rate = rospy.Rate(1)
    pub = rospy.Publisher("camera_trigger",Bool,queue_size=1)

    while not rospy.is_shutdown():
        
        rospy.loginfo(f"triggering camera : {camera_trigger}")
        # pub.publish(camera_trigger)
        rate.sleep()

def shutdown_callback(data):
    """
        The callback function to shutdown all active nodes
    """
    rospy.loginfo(f"Printing data {data.data}")
    if data.data == 'shutdown':
        rospy.signal_shutdown("Shutting down GPS")
    

def __sys_log():

    sub = rospy.Subscriber('shutdown',String,callback=shutdown_callback)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node("camera_trigger_node")

    thread1 = threading.Thread(target=get_gps_data)
    thread2 = threading.Thread(target=__sys_log)
    thread2.start()
    thread1.start()
    



# def shutdown_callback(data):

#     rospy.loginfo(f"Printing data {data.data}")
#     # if data.level == Log.FATAL and data.msg == 'shutdown':
#     #     rospy.signal_shutdown("Shutting down GPS")

# if __name__ == '__main__': 

#     rospy.init_node("camera_trigger")

#     camera_trigger = True
#     rate = rospy.Rate(1)
#     pub = rospy.Publisher("camera_trigger",Bool,queue_size=10)
#     sub = rospy.Subscriber('shutdown',String,callback=shutdown_callback)
#     while not rospy.is_shutdown():
        
#         rospy.loginfo(f"triggering camera : {camera_trigger}")
#         pub.publish(camera_trigger)
#         rate.sleep()
#     rospy.spin()
    
    

    