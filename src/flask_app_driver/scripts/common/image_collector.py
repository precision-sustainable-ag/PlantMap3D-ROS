import cv2, os, shutil, time
import numpy as np

class Collector():
	_shared_borg_state = {}

	def __new__(cls, *args, **kwargs):
		obj = super(Collector, cls).__new__(cls, *args, **kwargs)
		obj.__dict__ = cls._shared_borg_state
		return obj

	def initialize_queues(self, cd):
		self.preview_queue = cd.device.getOutputQueue(name="preview", maxSize=1, blocking=False)
		self.rgb_queue = cd.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
		self.left_queue = cd.device.getOutputQueue(name="left", maxSize=1, blocking=False)
		self.right_queue = cd.device.getOutputQueue(name="right", maxSize=1, blocking=False)
		self.depth_queue = cd.device.getOutputQueue(name="depth", maxSize=1, blocking=False)

	def find_file(self, img_type):
		folder = './images'
		for root, dirs, files in os.walk(folder):
			for file in files:
				if img_type in file:
					print(file)
					return file

	def get_frame(self, queue, img_type, depth=None):
		img_out = queue.tryGet()

		if img_out is not None:
			img_out = img_out.getCvFrame()
			encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 98]
			_, img_encoded = cv2.imencode('.jpg', img_out, encode_param)

			byte_stream = img_encoded.tobytes()

			return byte_stream

		return None

	def save_frames(self, depth):
		folder = './images'
		for filename in os.listdir(folder):
			file_path = os.path.join(folder, filename)
			try:
				if os.path.isfile(file_path) or os.path.islink(file_path):
					os.unlink(file_path)
				elif os.path.isdir(file_path):
					shutil.rmtree(file_path)
			except Exception as e:
				print('Failed to delete %s. Reason: %s' % (file_path, e))


		image_data = [
			{'img_type': 'depth', 'queue': self.depth_queue},
            {'img_type': 'right', 'queue': self.right_queue}, 
            {'img_type': 'left', 'queue': self.left_queue},
            {'img_type': 'rgb', 'queue': self.rgb_queue}, 
        ]

		def get_frames():
			last_sequence_number = None
			index = 0

			for data in image_data:
				# print(data.get('img_type'))
				img_out = data.get('queue').tryGet()

				if img_out is not None:
					if last_sequence_number == img_out.getSequenceNum() or index == 0:
						data['sequence_number'] = img_out.getSequenceNum()
						data['sensitivity'] = img_out.getSensitivity()
						data['exposure_time'] = img_out.getExposureTime()
						data['img_out'] = img_out

						last_sequence_number = img_out.getSequenceNum()
					else:
						print('sequence mismatch')
						return False
				else:
					print(data.get('img_type') + 'image is none')
					return False

				index += 1

			return True

		while not get_frames():
			print('continue')
			time.sleep(0.1)
			pass

		for data in image_data:
			if data.get('img_type') == "depth":
				# print(depth)
				img_out = data.get('img_out').getCvFrame()
				img_out = img_out.astype(np.uint16)
				cv2.imwrite('./images/{}_{}_{}_{}.png'.format(data.get('img_type'), data.get('sequence_number'), data.get('sensitivity'), data.get('exposure_time')), img_out)
			elif data.get('img_type') == 'rgb':
				with open("./images/{}_{}_{}_{}.jpg".format(data.get('img_type'), data.get('sequence_number'), data.get('sensitivity'), data.get('exposure_time')), "wb") as fw:
					fw.write(data.get('img_out').getData())
			else:
				cv2.imwrite('./images/{}_{}_{}_{}.jpg'.format(data.get('img_type'), data.get('sequence_number'), data.get('sensitivity'), data.get('exposure_time')), data.get('img_out').getCvFrame())
