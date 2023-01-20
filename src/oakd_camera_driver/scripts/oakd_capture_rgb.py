#!/usr/bin/env python3
"""
    This class is a driver for streaming the RGB pipeline for oakd cameras.
"""
import time, sys
import cv2
import depthai as dai
import numpy as np

class StartCameraStream():

	def __init__(self,ip,gps_flag):
	
		self.ip = ip
		self.gps_flag = gps_flag
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
	
	def start_camera_stream(self,gps_flag):

		self.gps_flag = gps_flag
		cam = dai.DeviceInfo(self.ip)
		with dai.Device(self.pipeline, cam) as device:	

			print("Connected")
			qRGB = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

			while True:

				frame = qRGB.tryGet()
				if frame is not None:
					
					self.get_image_frame(frame)
					cv2.imshow("cam", frame.getCvFrame())

				if cv2.waitKey(1) == ord('q'):
					break
	
	def get_image_frame(self, frame):
		"""
			This function will check for the GPS flag to be true and then return the latest frame
		"""
		if self.gps_flag == True :
			
			# return frame
			rospy.loginfo('gps flag is : '+self.gps_flag)
			rospy.loginfo('Returning Images')
		
		self.gps_flag = False
		rospy.loginfo('gps flag changed to : '+self.gps_flag)


		