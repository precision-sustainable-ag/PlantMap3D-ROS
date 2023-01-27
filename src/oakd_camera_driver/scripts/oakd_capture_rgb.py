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

	def __init__(self,ip,node_name):
		
		rospy.init_node('camera_driver')
		self.ip = ip
		self.node_name = node_name
		
		self.br =  CvBridge()

		self.pipeline = dai.Pipeline()
		self.pub = rospy.Publisher(self.node_name,Image,queue_size=10)

		self.camRgb = self.pipeline.create(dai.node.ColorCamera)
		self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
		self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
		self.camRgb.setPreviewSize(600,400)

		self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
		self.xoutRgb.setStreamName("rgb")
		self.xoutRgb.input.setBlocking(False)
		self.xoutRgb.input.setQueueSize(1)
		self.camRgb.preview.link(self.xoutRgb.input)

	
	def start_camera_stream_rgb(self):

		cam = dai.DeviceInfo(self.ip)
		with dai.Device(self.pipeline, cam) as device:	

			print("Connected")
			qRGB = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

			while True:

				frame = qRGB.tryGet()
				if frame is not None:
					self.pub.publish(self.br.cv2_to_imgmsg(frame.getCvFrame())) 
					cv2.imshow(self.node_name,frame.getCvFrame())
				if cv2.waitKey(1) == ord('q'):
					break
	


