import cv2, os, io
import numpy as np

class Collector():
	_shared_borg_state = {}

	def __new__(cls, *args, **kwargs):
		obj = super(Collector, cls).__new__(cls, *args, **kwargs)
		obj.__dict__ = cls._shared_borg_state
		return obj

	def encode_frame(self, img_out):
		if img_out is not None:

			encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 98]
			_, img_encoded = cv2.imencode('.jpg', img_out, encode_param)

			byte_stream = io.BytesIO(img_encoded)

			return byte_stream
	
	# def get_frame(rgb_queue,segmentation_queue):
	# 	rgb_in = rgb_queue.get()
	# 	rgb_out = rgb_in.getCvFrame()
	# 	segmentation_labels = segmentation_queue.get()
	# 	seg_labels = (np.array(segmentation_labels.getFirstLayerFp16()).reshape(128,128)).astype(np.uint8)
	# 	cv2.imwrite("rgb_img.jpg", rgb_out)
	# 	cv2.imwrite("seg_img.jpg", seg_labels)
	# 	return rgb_out, seg_labels

		
		

		

		


	