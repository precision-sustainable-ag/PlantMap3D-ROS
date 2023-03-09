#!/usr/bin/env python3 

import rospy 
from std_msgs.msg import String 
from rosgraph_msgs.msg import Log



if __name__ == '__main__':

    rospy.init_node("node_shutdown")

    pub = rospy.Publisher('shutdown',String,queue_size=10)

    rospy.loginfo("Sleeping for 20 secs")
    rospy.sleep(50)
    pub.publish("shutdown")
    rospy.loginfo("Shutting down system...")
    rospy.signal_shutdown("shutting down all nodes")
    rospy.sleep(1)