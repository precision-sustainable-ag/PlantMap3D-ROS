#!/usr/bin/env python3
"""
    This class is a driver for streaming the RGB pipeline for oakd cameras.
"""
import time, sys
import cv2
import depthai as dai
import numpy as np


class StartCameraStream():

	def __init__(self,ip):
	
		self.ip = ip
		
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
	
	def run_camera(self):
		cam = dai.DeviceInfo(self.ip)
		with dai.Device(self.pipeline, cam) as device:	
			print("Connected")
			qRGB = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
			while True:
				frame = qRGB.tryGet()
				if frame is not None:
					cv2.imshow("cam", frame.getCvFrame())
				if cv2.waitKey(1) == ord('q'):
					break

# if __name__ == '__main__':
	
#     # enter IP address of the cameras in the input field.
# 	cmd_cam = StartCameraStream("")
# 	cmd_cam.run_camera()
		