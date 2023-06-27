import cv2, os, io
import numpy as np

class Collector():
	_shared_borg_state = {}

	def __new__(cls, *args, **kwargs):
		obj = super(Collector, cls).__new__(cls, *args, **kwargs)
		obj.__dict__ = cls._shared_borg_state
		return obj

	def find_file(self, img_type):
		folder = './images'
		for root, dirs, files in os.walk(folder):
			for file in files:
				if img_type in file:
					print(file)
					return file	

	def encode_frame(self, img_out):
		if img_out is not None:

			encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 98]
			_, img_encoded = cv2.imencode('.jpg', img_out, encode_param)

			byte_stream = io.BytesIO(img_encoded)

			return byte_stream
