# import io
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
print(sys.path)
from multiprocessing import Process, Queue
from common.camera_data_loader import get_camera_nodes
from common.check_camera import check_camera_connection
from common.camera_queuemaker import CameraDevice


rgb_q = Queue()
cam_device = CameraDevice()
cam_process = Process(target=cam_device.oakd_camera_reader, args=(rgb_q,))
cam_process.start()

frame = rgb_q.get()
print(frame.shape)
