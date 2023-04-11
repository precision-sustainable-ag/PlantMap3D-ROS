#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
import numpy as np
import os 
import sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))


class TestImagePublisher():

    def __init__(self,node_name:str,cameraid:str):
        
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.wait_for_service("camera_location_service")
        self.__topic_name = 'camera_data'
        self.pub = rospy.Publisher(self.__topic_name,numpy_msg(PM3DCameraData),queue_size=1)
        self.sub = None
        self.cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        self.cam_location_req = PM3DCameraLocationRequest()
        self.camera_id = int(cameraid)


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

        # msg.camera_id = self.get_camera_id(self.node_name)
        msg.camera_id = self.camera_id
        # Updating camera  coordinates based on camera location 
        self.cam_location_req.gpscoords = [data.latitude,data.longitude]
        self.cam_location_req.gpsheading = data.gps_heading
        self.cam_location_req.cameraid = msg.camera_id

        cam_location_response = self.cam_location(self.cam_location_req)
        msg.gps_data.latitude = cam_location_response.newgpscoords[0]
        msg.gps_data.longitude = cam_location_response.newgpscoords[1]
        msg.gps_data.gps_heading = data.gps_heading
        
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
    

    