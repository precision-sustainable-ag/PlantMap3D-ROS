#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Bool, String
from rosgraph_msgs.msg import Log
import threading

# def get_gps_data():

#     camera_trigger = True
#     rate = rospy.Rate(10)
#     pub = rospy.Publisher("camera_trigger",Bool,queue_size=1)

#     while not rospy.is_shutdown():
        
#         rospy.loginfo(f"triggering camera : {camera_trigger}")
#         pub.publish(camera_trigger)
#         rate.sleep()

# def shutdown_callback(data):

#     if data.level == Log.FATAL and data.msg == 'shutdown':
#         rospy.signal_shutdown("Shutting down GPS")
    

# def __sys_log():

#     rospy.Subscriber('/rosout',Log,callback=shutdown_callback)
#     rospy.spin()

# if __name__ == '__main__':

#     rospy.init_node("camera_trigger")

#     thread1 = threading.Thread(target=get_gps_data)
#     thread2 = threading.Thread(target=__sys_log)

#     thread1.start()
#     thread2.start()

#     thread1.join()
#     thread2.join()

#     rospy.spin()

def shutdown_callback(data):

    rospy.loginfo(f"Printing data {data}")
    # if data.level == Log.FATAL and data.msg == 'shutdown':
    #     rospy.signal_shutdown("Shutting down GPS")

if __name__ == '__main__': 

    rospy.init_node("camera_trigger")

    camera_trigger = True
    rate = rospy.Rate(1)
    pub = rospy.Publisher("camera_trigger",Bool,queue_size=10)
    rospy.Subscriber('/rosout',Log,callback=shutdown_callback)
    while not rospy.is_shutdown():
        
        rospy.loginfo(f"triggering camera : {camera_trigger}")
        pub.publish(camera_trigger)
        rate.sleep()
    
    

    