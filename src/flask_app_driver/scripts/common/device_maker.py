import depthai as dai
import cv2
import os
import numpy as np 

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
        self.__blob_path = "/model.blob"
        self.nn_path = os.path.join('models' + self.__blob_path)

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

    def get_camera_image(self):
        rgb_out = None
        with dai.Device(self.pipeline,self.cam) as self.device:
            self.status = "active"
            rgb_queue = self.device.getOutputQueue("rgb", maxSize=1, blocking=True)
            segmentation_queue = self.device.getOutputQueue("seg out",1,False)

            rgb_in = rgb_queue.get()
            rgb_out = rgb_in.getCvFrame()
            segmentation_labels = segmentation_queue.get()
            seg_labels = (np.array(segmentation_labels.getFirstLayerFp16()).reshape(128,128)).astype(np.uint8)
            cv2.imwrite("rgb_img.jpg", rgb_out)
            cv2.imwrite("seg_img.jpg", seg_labels)
  
            # cv2.imshow("rgb",rgb_out)

            # if cv2.waitKey(1) == ord('q'):
            #     break

        return rgb_out, seg_labels


        

