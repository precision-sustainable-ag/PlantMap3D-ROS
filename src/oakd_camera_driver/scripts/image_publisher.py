#!/usr/bin/env python3

import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
from std_msgs.msg import Header
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
import numpy as np
import cv2
import os 
import sys
import glob
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class TestImagePublisher():
    """
    This class reads the synthetic data and acts as an image publisher
    This node is subscribed to camera trigger and is subscribed by the height map pipeline
    """
    def __init__(self,node_name:str,cameraid:int):
        
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.wait_for_service("camera_location_service")
        self.__topic_name = 'camera_data'
        self.pub = rospy.Publisher(self.__topic_name,numpy_msg(PM3DCameraData),queue_size=10)
        self.sub = None
        self.cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        self.cam_location_req = PM3DCameraLocationRequest()
        self.camera_id = int(cameraid)
        
        rospack_testset = rospkg.RosPack()
        __test_rgbpath = rospack_testset.get_path('oakd_camera_driver') + '/tests/rgb/'
        __test_depthpath = rospack_testset.get_path('oakd_camera_driver') + '/tests/depth/'
        __test_segmentationpath = rospack_testset.get_path('oakd_camera_driver') + '/tests/segmentation/'
        
        try :

            def get_image_list(image_files):

                image_list = []

                for image_file in image_files:
                    
                    if not os.path.isfile(image_file):

                        raise FileNotFoundError(f"Given {image_file} does not exists..")
                    else:
                        img = cv2.imread(image_file)
                        image_list.append(img)
                return image_list
            
            rgb_image_files = glob.glob(os.path.join(__test_rgbpath,'*.jpg'))
            depth_image_file = glob.glob(os.path.join(__test_depthpath,'*.png'))
            segmentation_image_file = glob.glob(os.path.join(__test_segmentationpath,'*.png'))

            test_rgb_images = get_image_list(rgb_image_files)
            test_depth_images = get_image_list(depth_image_file)
            test_segmentation_images = get_image_list(segmentation_image_file)

        except FileNotFoundError as e:
            print(e)
        self.test_rgb_images = test_rgb_images
        self.test_depth_images = test_depth_images
        self.test_segmentation_images = test_segmentation_images


    def callback(self,data):
        
        if data.camera_trigger == True:
            """
            When camera trigger is true, publish camera image data
            """
            image_size = 518
            array_data = cv2.resize(np.array(self.test_rgb_images[1]),dsize=(image_size,image_size))
            depth_data = cv2.resize(np.array(self.test_depth_images[1]),dsize=(image_size,image_size))
            segmentation_label_arr = cv2.resize(np.array(self.test_segmentation_images[1]),dsize=(108,108))
            
            header = Header()
            header.stamp = rospy.Time.now()
            msg = PM3DCameraData()
            msg.header = header
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
            # updating coordinates
            msg.gps_data.latitude = cam_location_response.newgpscoords[0]
            msg.gps_data.longitude = cam_location_response.newgpscoords[1]
            msg.gps_data.gps_heading = data.gps_heading

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
    

    