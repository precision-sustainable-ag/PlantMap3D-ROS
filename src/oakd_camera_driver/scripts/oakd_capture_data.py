#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import time
import threading
import depthai as dai
import numpy as np
import cv2
import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
from std_msgs.msg import Header
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
from std_msgs.msg import Bool
import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class PM3DCameraDataPublisher():
    """
    This class is a driver for streaming the PM3DCamera Data driver for oakd cameras.
    This function returns RGBD and segmentation labels.
    """
    def __init__(self,ip,node_name,topic_name,test_flag):
        
        self.ip = ip
        self.node_name = node_name
        self.topic_name = topic_name
        self.test_flag = test_flag

        rospy.init_node(node_name)
        rospy.loginfo('Creating new camera node')

        self.pub = rospy.Publisher(topic_name,numpy_msg(PM3DCameraData),queue_size=6)
        self.camera_data_msg = PM3DCameraData()
        
        self.__blob_path = "/small1024_2021_4_8shaves_only_dt_and_is.blob"

        self.nn_shape = (1024,1024)

        self.nn_path = os.path.join(os.getcwd() +'/models' + self.__blob_path)
        print(self.nn_path)

        self.pipeline = dai.Pipeline()

        self.pipeline.setOpenVINOVersion(version=dai.OpenVINO.VERSION_2021_4)
        
        print("Creating color camera node...")
        self.RGB_Node = self.pipeline.createColorCamera()
        self.RGB_Node.setPreviewSize(self.nn_shape[0], self.nn_shape[1])
        self.RGB_Node.setInterleaved(False)
        self.RGB_Node.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.RGB_Node.setBoardSocket(dai.CameraBoardSocket.RGB)

        self.RGB_Out=self.pipeline.create(dai.node.XLinkOut)
        self.RGB_Out.setStreamName("rgb")
        self.RGB_Out.input.setBlocking(False)
        self.RGB_Out.input.setQueueSize(1)
        self.RGB_Node.preview.link(self.RGB_Out.input)
        print("Done")

        print("Creating neural network node...")
        self.nn_node = self.pipeline.create(dai.node.NeuralNetwork)
        self.nn_node.setBlobPath(self.nn_path)
        self.nn_node.input.setQueueSize(1)
        self.nn_node.input.setBlocking(False)

        self.RGB_Node.preview.link(self.nn_node.input)

        self.seg_out = self.pipeline.create(dai.node.XLinkOut)
        self.seg_out.setStreamName("seg out")
        self.nn_node.out.link(self.seg_out.input)
        print("Done")

        print("Creating depth channel...")
        self.monoL = self.pipeline.create(dai.node.MonoCamera)
        self.monoR = self.pipeline.create(dai.node.MonoCamera)
        self.depth = self.pipeline.create(dai.node.StereoDepth)

        self.depth_out = self.pipeline.create(dai.node.XLinkOut)
        self.depth_out.setStreamName("depth")
        self.monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)

        self.monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.monoL.out.link(self.depth.left)
        self.monoR.out.link(self.depth.right)
        self.depth.depth.link(self.depth_out.input)
        print("Done")

        
        self.cam = dai.DeviceInfo(ip)
        with dai.Device(self.pipeline,self.cam) as self.device:
            self.enable_camera()
        

    def enable_camera(self):
        ## Start the camera stream
        depth_out = self.device.getOutputQueue("depth",1,False)
        segmentation_queue = self.device.getOutputQueue("seg out",1,False)
        rgb_queue = self.device.getOutputQueue("rgb",1,False)
        while True:
            segmentation_labels = segmentation_queue.get()
        
            segmentation_labels = np.array(segmentation_labels.getFirstLayerInt32()).reshape(self.nn_shape[0],self.nn_shape[1])
        
            self.out = [None,None,None]
            rgb_data = rgb_queue.get()
            rgb_img = np.array(rgb_data.getCvFrame())
            depth_data = depth_out.get()
            depth_img = np.array(depth_data.getCvFrame())
            depth_img = (depth_img * (255/self.depth.initialConfig.getMaxDisparity())).astype(np.uint8)

            self.out[0] = rgb_img 
            self.out[1] = segmentation_labels 
            self.out[2] = depth_img 

            cv2.imshow(f"{self.node_name}",rgb_img)
            
            self.camera_data_msg.rgb_data = rgb_img.flatten().tolist()
            self.camera_data_msg.depth_map = depth_img.flatten().tolist()
            self.camera_data_msg.segmentation_labels = segmentation_labels.flatten().tolist()

            self.camera_data_msg.rgb_dims = rgb_img.shape 
            self.camera_data_msg.depth_map_dims = depth_img.shape 
            self.camera_data_msg.segmentation_label_dims = segmentation_labels.shape

            self.pub.publish(self.camera_data_msg)
            if cv2.waitKey(1) == ord('q'):
                break

    def camera_trigger_listener(self):

        rospy.Subscriber('camera_trigger',Bool,callback=self.callback)
        rospy.spin()

    def dummy_function(self):

        return test_flag
    
    def run_threads():
        t1 = threading.Thread(target=self.resolve_domain)

    def oakd_callback(self,data):

        return 0 
    
    def run_camera_trigger_subscriber(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',PM3DGPSData,callback=self.oakd_callback)
            rospy.spin()

if __name__ == "__main__":
    
    cmd_rec = sys.argv[1:]
    seg_pipeline = PM3DCameraDataPublisher(cmd_rec[0],cmd_rec[1],cmd_rec[2])
    seg_pipeline.enable_camera()
