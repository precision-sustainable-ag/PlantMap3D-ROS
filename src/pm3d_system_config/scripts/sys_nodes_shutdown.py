#!/usr/bin/env python3 
"""
@author: MathewAaron
"""
import rospy 
from std_msgs.msg import String 
from rosgraph_msgs.msg import Log
import rosnode

"""
    This function acts as a global shutdown node

    This node can be called incase a critical shutdown is needed.
"""
def shutdown_all_nodes():

    nodes_list = rosnode.get_node_names()

    for node_name in nodes_list:

        if node_name != rospy.get_name():
            rospy.loginfo(f"Shutting down node : {node_name}")
            rosnode.kill_nodes([node_name])

if __name__ == '__main__':

    rospy.init_node("node_shutdown")

    pub = rospy.Publisher('shutdown',String,queue_size=10)

    # Sending shutdown signal after 5 secs, just for testing.
    rospy.logwarn("Shutting down in 5 secs")
    rospy.sleep(5)
    #pub.publish("shutdown")
    shutdown_all_nodes()
    rospy.loginfo("Shutting down system...")
    rospy.signal_shutdown("shutting down all nodes")
    rospy.sleep(1)