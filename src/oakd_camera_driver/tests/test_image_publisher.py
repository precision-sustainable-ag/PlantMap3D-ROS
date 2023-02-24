#!/usr/bin/env python3

import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from std_msgs.msg import Bool

import numpy as np
np.random.seed(45)
class TestImagePublisher():

    def __init__(self,topic_name="camera_1"):
        
        rospy.init_node("camera_data")
        self.image_name = np.random.randint(0,255,(1024,1024,3),dtype=np.int64)
        self.node_name = "camera_data"
        self.topic_name = topic_name
        self.array_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        self.depth_data = np.random.randint(0,30,(128,128,3),dtype=np.int64)
        self.segmentation_label_arr = np.random.randint(0,4,(128,128,3),dtype=np.int64)
        self.array_data_flag = self.array_data
        self.depth_data_flag = self.depth_data 
        self.segmentation_label_arr_flag = self.segmentation_label_arr 
        self.pub = rospy.Publisher(topic_name,numpy_msg(PM3DCameraData),queue_size=1)
        self.run()

    def callback(self,data):
        
        print(self.image_name)
        
        msg = PM3DCameraData()
        msg.rgb_data = self.array_data.flatten().tolist()
        msg.depth_map = self.depth_data.flatten().tolist()
        msg.segmentation_labels = self.segmentation_label_arr.flatten().tolist()

        msg.rgb_dims = self.array_data.shape
        msg.depth_map_dims = self.depth_data.shape
        msg.segmentation_label_dims = self.segmentation_label_arr.shape

        rospy.loginfo("Node name : %s" ,rospy.get_name())
        if data.data == True:
            """
            When camera trigger is true, publish camera image data
            """
            rospy.loginfo("Publishing Camera Data")
            self.pub.publish(msg)
        else:
            rospy.loginfo("Not Publishing")

    def return_current_camera_data(self):

        return self.array_data_flag, self.depth_data_flag, self.segmentation_label_arr_flag
        
    def run(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',Bool,callback=self.callback)
            rospy.spin()



if __name__ == '__main__':

    print("in system")
    try : 
        camera_obj = TestImagePublisher()

    except rospy.ROSInterruptException:
        print("out system")