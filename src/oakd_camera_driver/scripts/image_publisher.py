#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
import numpy as np
import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))


class TestImagePublisher():

    def __init__(self,node_name,topic_name):
        
        self.node_name = node_name
        rospy.init_node(node_name)
        self.topic_name = topic_name
        self.pub = rospy.Publisher(topic_name,numpy_msg(PM3DCameraData),queue_size=1)
        self.sub = None

    def callback(self,data):
        
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

        msg.gps_data.latitude = data.latitude
        msg.gps_data.longitude = data.longitude
        msg.gps_data.gps_heading = data.gps_heading
        msg.camera_id = 0
        if data.camera_trigger == True:
            """
            When camera trigger is true, publish camera image data
            """
            rospy.loginfo(f"Publishing {self.node_name} Data")
            self.pub.publish(msg)
        else:
            rospy.loginfo("Not Publishing")

    def run(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',PM3DGPSData,callback=self.callback)
            rospy.spin()
    
          

if __name__ == '__main__':

    cmd_rec = sys.argv[1:]
    camera_obj = TestImagePublisher(cmd_rec[0],cmd_rec[1])
    camera_obj.run()
    

    