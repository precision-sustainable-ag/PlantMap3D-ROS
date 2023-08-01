#!/usr/bin/env python3
"""
@author: MathewAaron
"""
# import roslib; roslib.load_manifest('oakd_camera_driver')
# roslib.load_manifest('camera_trigger_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
from std_msgs.msg import Header
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
import numpy as np
import cv2
import os 
import sys
import glob
from cv_bridge import CvBridge

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
        self.__image_topic_name = 'camera{}_image'.format(cameraid)
        self.pub = rospy.Publisher(self.__topic_name,numpy_msg(PM3DCameraData),queue_size=10)
        self.image_pub = rospy.Publisher(self.__image_topic_name,numpy_msg(Image),queue_size=6)
        self.sub = None
        self.cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        self.cam_location_req = PM3DCameraLocationRequest()
        self.camera_id = int(cameraid)
        self.camera_name = node_name
        # Getting file path for test image set
        rospack_testset = rospkg.RosPack()
        __test_rgbpath = rospack_testset.get_path('oakd_camera_driver') + '/test/rgb/'
        __test_depthpath = rospack_testset.get_path('oakd_camera_driver') + '/test/depth/'
        __test_segmentationpath = rospack_testset.get_path('oakd_camera_driver') + '/test/segmentation/'
        
        # Getting file path for image save set
        rospack_datapath = rospkg.RosPack()
        self.__rgbpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/rgb/'
        self.__depthpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/depth/'
        self.__segmentationpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/segmentation/'

        self.bridge =  CvBridge()
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
            array_data = cv2.resize(np.array(self.test_rgb_images[0]),dsize=(image_size,image_size))
            depth_data = cv2.resize(np.array(self.test_depth_images[0]),dsize=(image_size,image_size))
            segmentation_label_arr = cv2.resize(np.array(self.test_segmentation_images[0]),dsize=(518,518))

            
            time_stamp = rospy.Time.now()
            header = Header()
            header.stamp = time_stamp
            msg = PM3DCameraData()
            msg.header = header
            # msg.camera_name = self.camera_name
            # msg.rgb_data = array_data.flatten().tolist()
            # msg.depth_map = depth_data.flatten().tolist()
            # msg.segmentation_labels = segmentation_label_arr.flatten().tolist()

            # msg.rgb_dims = array_data.shape
            # msg.depth_map_dims = depth_data.shape
            # msg.segmentation_label_dims = segmentation_label_arr.shape
            # msg.height_map_dims = depth_data.shape
            # msg.camera_id = self.get_camera_id(self.node_name)
            
            msg.camera_name = self.camera_name
            msg.header = header
            msg.rgb_data = self.bridge.cv2_to_imgmsg(array_data,"bgr8") if array_data is not None else None
            msg.depth_map = self.bridge.cv2_to_imgmsg(depth_data,"bgr8") if depth_data is not None else None
            msg.segmentation_labels = self.bridge.cv2_to_imgmsg(segmentation_label_arr,"bgr8")
            
            msg.camera_id = self.camera_id
            # Updating camera  coordinates based on camera location 
            self.cam_location_req.gpscoords = [data.latitude,data.longitude]
            self.cam_location_req.gpsheading = data.gps_heading
            self.cam_location_req.cameraid = self.camera_id

            cam_location_response = self.cam_location(self.cam_location_req)
            # updating coordinates
            msg.latitude = float(cam_location_response.newgpscoords[0])
            msg.longitude = float(cam_location_response.newgpscoords[1])
            msg.gps_heading = float(data.gps_heading)


            # saving image data
            self.__image_save(array_data,self.__rgbpath,self.camera_id,time_stamp)
            self.__image_save(depth_data,self.__depthpath,self.camera_id,time_stamp)
            self.__image_save(segmentation_label_arr,self.__segmentationpath,self.camera_id,time_stamp)

            rospy.loginfo(f"Publishing {self.node_name} Data")
            self.pub.publish(msg)
            self.image_pub.publish(msg.rgb_data)
        else:
            rospy.loginfo("Not Publishing")


    def __image_save(self,data,__path,cameraid, timestamp):

        image_name = "image_"+str(cameraid)+"_"+str(timestamp)+".png"
        cv2.imwrite(__path+image_name,data)
    
    def run(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',PM3DGPSData,callback=self.callback)
            rospy.spin()
    
          

if __name__ == '__main__':

    cmd_rec = sys.argv[1:]
    camera_obj = TestImagePublisher(cmd_rec[0],cmd_rec[1])
    camera_obj.run()
    

    