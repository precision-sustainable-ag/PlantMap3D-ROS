#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Bool
import numpy as np
import cv2
import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))

"""
PM3DCameraData :
    int64[] rgb_data
    int64[] depth_map
    int64[] segmentation_labels
    int64[] rgb_dims
    int64[] depth_map_dims
    int64[] segmentation_label_dims
    int64[] height_map
    int64[] height_map_dims
    float32[] biomass_estimate
"""


class TestImagePublisher():

    def __init__(self,image_name,node_name,topic_name):
        
        rospy.init_node(node_name)
        self.image_name = cv2.imread(image_name)
        self.node_name = node_name
        self.topic_name = topic_name
        self.pub = rospy.Publisher(topic_name,numpy_msg(PM3DCameraData),queue_size=1)
        self.run()

    def callback(self,data):
        
        print(self.image_name)
        array_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)
        msg = PM3DCameraData()
        msg.rgb_data = array_data.flatten().tolist()
        msg.depth_map = depth_data.flatten().tolist()
        msg.segmentation_labels = segmentation_label_arr.flatten().tolist()

        msg.rgb_dims = array_data.shape
        msg.depth_map_dims = depth_data.shape
        msg.segmentation_label_dims = segmentation_label_arr.shape

        rospy.loginfo("Node name : %s" ,rospy.get_name())
        if data.data == True:
            """
            When camera trigger is true, publish camera image data
            """
            rospy.loginfo("Publishing Camera Data")
            self.pub.publish(msg)
        else:
            rospy.loginfo("Not Publishing")

    def run(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',Bool,callback=self.callback)
            rospy.spin()



if __name__ == '__main__':

    #talker()
    print("in system")
    # cmd_rec = sys.argv[1:]
    # camera_obj = DummyImagePublisher(cmd_rec[0],cmd_rec[1],cmd_rec[2])
    try : 
        camera_obj = TestImagePublisher(sys.argv[0],sys.argv[1],sys.argv[2])

    except rospy.ROSInterruptException:
        print("out system")

    