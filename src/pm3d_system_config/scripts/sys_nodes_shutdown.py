#!/usr/bin/env python3 

import rospy 
from std_msgs.msg import String 
from rosgraph_msgs.msg import Log



if __name__ == '__main__':

    rospy.init_node("node_shutdown")

    # log = Log()
    # log.level = log.FATAL
    # log.msg = 'shutdown'

    # for i in range(5):
    #     rospy.loginfo(log)
    #     rospy.sleep(1)

    pub = rospy.Publisher('shutdown',String,queue_size=10)

    rate = rospy.Rate(1)