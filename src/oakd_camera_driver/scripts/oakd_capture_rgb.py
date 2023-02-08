#!/usr/bin/env python3
"""
    This class is a driver for streaming the RGB pipeline for oakd cameras.
"""

import cv2
import depthai as dai
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time, sys 
import os

class StartCameraStream():

	def __init__(self,ip,node_name,topic_name):
		
		

		self.ip = ip
		self.node_name = node_name
		self.topic_name = topic_name
		rospy.init_node(node_name)
		rospy.loginfo('Creating new camera node')
		self.pub = rospy.Publisher(self.topic_name,Image,queue_size=10)
		
		self.br =  CvBridge()

		self.pipeline = dai.Pipeline()
		
		self.camRgb = self.pipeline.create(dai.node.ColorCamera)
		self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
		self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
		self.camRgb.setPreviewSize(600,400)

		self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
		self.xoutRgb.setStreamName("rgb")
		self.xoutRgb.input.setBlocking(False)
		self.xoutRgb.input.setQueueSize(1)
		self.camRgb.preview.link(self.xoutRgb.input)

		self.monoRight = self.pipeline.create(dai.node.MonoCamera)
		self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
		self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
		self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
		self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
		self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

		self.Depth = self.pipeline.create(dai.node.StereoDepth)
		self.Depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
		self.Depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
		self.Depth.setLeftRightCheck(True)
		self.Depth.setExtendedDisparity(False)
		self.Depth.setSubpixel(False)
		
		self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
		self.xoutDepth.setStreamName("disparity")
		self.monoRight.out.link(self.Depth.right)
		self.monoLeft.out.link(self.Depth.left)
		self.Depth.disparity.link(self.xoutDepth.input)

		self.get_camera_data()

	
	def get_camera_data(self):

		cam = dai.DeviceInfo(self.ip)
		with dai.Device(self.pipeline, cam) as device:	

			print(f"Connected to {self.node_name}")
			qRGB = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
			qDepth = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)

			while True:

				frame = qRGB.tryGet()
				inDepth = qDepth.tryGet()

				if inDepth is not None:
					dframe = inDepth.getFrame()
					# dframe = (dframe * init_dsp).astype(np.uint8)
					# cv2.imshow("Depth", dframe)

				if frame is not None:
					
					cv2.imshow(self.node_name,frame.getCvFrame())
					self.pub.publish(self.br.cv2_to_imgmsg(frame.getCvFrame(),"bgr8"))
					time.sleep(2) 

				if cv2.waitKey(1) == ord('q'):
					break
	
if __name__ == '__main__':

	cmd_rec = sys.argv[1:]
	camera_obj = StartCameraStream(cmd_rec[0],cmd_rec[1],cmd_rec[2])