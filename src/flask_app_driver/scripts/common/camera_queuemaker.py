import depthai as dai
import cv2
import os
import numpy as np 
import sys
from multiprocessing import Process, Queue
import threading
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
print(sys.path)
from common.camera_data_loader import get_camera_nodes

class CameraDevice():
    status = "inactive"
    _shared_borg_state = {}
    
    def __new__(cls, *args, **kwargs):
        obj = super(CameraDevice, cls).__new__(cls, *args, **kwargs)
        obj.__dict__ = cls._shared_borg_state
        return obj

    def upload_pipeline(self, ip_address): 
        self.ip_address = ip_address    

        # Create pipeline
        self.pipeline = dai.Pipeline()
        # self.__blob_path = "/model.blob"
        # self.nn_path = os.path.join('models' + self.__blob_path)

        self.nn_path = "/home/plantmap3d/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/flask_app_driver/scripts/models/model.blob"
        # Connect to the camera using the provided IP address
        self.cam = dai.DeviceInfo(self.ip_address)

        # Define source and output
        print("Creating color camera node...")
        self.RGB_Node = self.pipeline.createColorCamera()
        self.RGB_Out=self.pipeline.create(dai.node.XLinkOut)

        self.RGB_Out.setStreamName("rgb")
        # Properties
        self.RGB_Node.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        self.RGB_Node.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.RGB_Node.setPreviewSize(1024,1024)

        self.RGB_Out.input.setBlocking(False)
        self.RGB_Out.input.setQueueSize(1)

        # Linking
        self.RGB_Node.preview.link(self.RGB_Out.input)
        print("Done creating RGB node")

        print("Creating neural network node...")
        self.nn_node = self.pipeline.create(dai.node.NeuralNetwork)
        self.nn_node.setBlobPath(self.nn_path)
        self.nn_node.input.setQueueSize(1)
        self.nn_node.input.setBlocking(False)
        self.RGB_Node.preview.link(self.nn_node.input)

        self.seg_out = self.pipeline.create(dai.node.XLinkOut)
        self.seg_out.setStreamName("seg out")
        self.nn_node.out.link(self.seg_out.input)

    def close_pipeline(self):
        self.device.close()
        self.status = "inactive"

    def make_queues(self, rgb_q):
        with dai.Device(self.pipeline,self.cam) as self.device:
            self.status = "active"
            rgb_queue = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
            rgb_queue
            segmentation_queue = self.device.getOutputQueue("seg out",1,False)
            print("Getting frames")
            rgb_in = rgb_queue.get()
            rgb_out = rgb_in.getCvFrame()
            segmentation_labels = segmentation_queue.get()
            seg_labels = (np.array(segmentation_labels.getFirstLayerFp16()).reshape(128,128)).astype(np.uint8)
            path = "/home/plantmap3d/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/flask_app_driver/scripts/common/"
            cv2.imwrite(path +"rgb_img.jpg", rgb_out)
            cv2.imwrite(path + "seg_img.jpg", seg_labels)
            print("Putting frame in queue")
            rgb_q.put(rgb_out)
            # seg_q.put(seg_labels)


    def oakd_camera_reader(self, rgb_q):
        camera_id =3
        node_data, node_names = get_camera_nodes()
        camera_ip = node_data[f"camera_{camera_id}"]['ip']
        print(camera_ip)
        self.upload_pipeline(camera_ip)
        self.make_queues(rgb_q )


