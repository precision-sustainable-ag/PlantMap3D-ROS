#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import time
import threading
import depthai as dai
import numpy as np
import cv2
import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from oakd_camera_driver.msg import PM3DCameraData
from camera_trigger_driver.msg import PM3DGPSData
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from camera_location_module.srv import PM3DCameraLocation, PM3DCameraLocationRequest
from std_msgs.msg import Bool
import glob
import os, sys
import queue
from cv_bridge import CvBridge
os.chdir(os.path.dirname(os.path.abspath(__file__)))

nISO = 19
ISO = np.logspace(1.25, 2, num=nISO+1, endpoint=True, base=40, dtype=int)
SS = [313, 400, 500, 625, 800, 1000]
T = 15
B = 12
dT = 1    #1.99
nQ = 4
w = np.flip(np.logspace(1, 0, num=nQ, endpoint=True))


class PM3DCameraDataPublisher():
    """
    This class is a driver for streaming the PM3DCamera Data driver for oakd cameras.
    This function returns RGBD and segmentation labels.
    """
    def __init__(self,ip,node_name:str,cameraid:int):
        
        rospy.init_node(node_name)
        rospy.loginfo('Creating new camera node')
        rospy.wait_for_service("camera_location_service")
        rospy.on_shutdown(self.__shutdown)
        self.ip = ip
        self.node_name = node_name
        self.topic_name = 'camera{}_data'.format(cameraid)
        self.image_topic_name = 'camera{}_image'.format(cameraid)
        self.test_flag = False
        self.gps_data = None
        self.camera_name = node_name

        self.pub = rospy.Publisher(self.topic_name,numpy_msg(PM3DCameraData),queue_size=6)
        self.image_pub = rospy.Publisher(self.image_topic_name,numpy_msg(Image),queue_size=6)
        self.camera_data_msg = PM3DCameraData()
        self.cam_location = rospy.ServiceProxy("camera_location_service",PM3DCameraLocation)
        self.cam_location_req = PM3DCameraLocationRequest()

        # Temporary camera id
        self.camera_id = int(cameraid)
        self.pipeline = dai.Pipeline()
        self.pipeline.setOpenVINOVersion(version=dai.OpenVINO.VERSION_2021_4)

        self.__blob_path = "/larger.blob"
        self.nn_shape = (1024,1024)
        self.nn_path = os.path.join(os.getcwd() +'/models' + self.__blob_path)
        print(self.nn_path)

        self.RGB_Node = self.pipeline.createColorCamera()
        self.RGB_Node.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        self.RGB_Node.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.RGB_Node.setPreviewSize(1024,1024)
        self.RGB_Node.setVideoSize(2048, 2048)
        self.RGB_Node.setInterleaved(False)
        # self.RGB_Node.setSharpness(0)     # range: 0..4, default: 1    
        # self.RGB_Node.setLumaDenoise(0)   # range: 0..4, default: 1    
        # self.RGB_Node.setChromaDenoise(4) # range: 0..4, default: 1

        script = self.pipeline.createScript()
        self.RGB_Node.video.link(script.inputs['isp'])
        script.inputs['isp'].setBlocking(False)

        script.setScript("""
            import datetime
            import time
            mark = datetime.datetime.now()
            while True:
                frame = node.io['isp'].get()
                now = datetime.datetime.now()
                if (now - mark).total_seconds() > 5:
                    mark = now
                    num = frame.getSequenceNum()
                    node.io['frame'].send(frame)
                time.sleep(0.01)
        """)

        
        # manip = self.pipeline.create(dai.node.ImageManip)
        # manip.initialConfig.setResize(400,400)
        # manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        # self.RGB_Node.preview.link(manip.inputImage)
        
        self.RGB_Out=self.pipeline.create(dai.node.XLinkOut)
        self.RGB_Out.setStreamName("rgb")
        # self.RGB_Node.video.link(self.RGB_Out.input)
        # manip.out.link(self.RGB_Out.input)
        script.outputs['frame'].link(self.RGB_Out.input)

        self.nn_node = self.pipeline.create(dai.node.NeuralNetwork)
        self.nn_node.setBlobPath(self.nn_path)
        self.nn_node.input.setBlocking(False)
        self.nn_node.input.setQueueSize(1)
        self.RGB_Node.preview.link(self.nn_node.input)

        self.seg_out = self.pipeline.create(dai.node.XLinkOut)
        self.seg_out.setStreamName("seg out")
        self.nn_node.out.link(self.seg_out.input)

        self.monoL = self.pipeline.create(dai.node.MonoCamera)
        self.monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)

        self.monoR = self.pipeline.create(dai.node.MonoCamera)
        self.monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.depth.setLeftRightCheck(True)
        self.depth.setExtendedDisparity(False)
        self.depth.setSubpixel(False)
        self.monoL.out.link(self.depth.left)
        self.monoR.out.link(self.depth.right)

        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight.setStreamName("right")
        self.monoR.out.link(self.xoutRight.input)

        self.xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        self.xoutLeft.setStreamName("left")
        self.monoL.out.link(self.xoutLeft.input)

        self.depth_out = self.pipeline.create(dai.node.XLinkOut)
        self.depth_out.setStreamName("depth")
        self.depth.depth.link(self.depth_out.input)

        fps = 3
        self.RGB_Node.setFps(fps)
        self.monoR.setFps(fps)
        self.monoL.setFps(fps)

        #self.RGB_Node.setIsp3aFps(1)
        #self.monoR.setIsp3aFps(1)
        #self.monoL.setIsp3aFps(1)

        self.xin1 = self.pipeline.create(dai.node.XLinkIn)
        self.xin1.setNumFrames(1)   
        self.xin1.setMaxDataSize(1) 
        self.xin1.setStreamName("controlr") 
        self.xin1.out.link(self.RGB_Node.inputControl) 
        
        self.xin2 = self.pipeline.create(dai.node.XLinkIn)   
        self.xin2.setNumFrames(1)
        self.xin2.setMaxDataSize(1) 
        self.xin2.setStreamName("controlm") 
        self.xin2.out.link(self.monoL.inputControl)   
        self.xin2.out.link(self.monoR.inputControl)

        rospack_datapath = rospkg.RosPack()
        self.__rgbpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/rgb/'
        self.__depthpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/depth/'
        self.__segmentationpath = rospack_datapath.get_path('data_saver_driver') + '/camera_data/right/'
        
        self.shutdown_flag = False
        self.cam = dai.DeviceInfo(ip)
        self.bridge =  CvBridge()

        print("\n\n !!!!! CAMERA SETUP COMPLETE !!!!! \n\n")
        
    def __shutdown(self):

        self.shutdown_flag = True

    def enable_camera(self):

        config = dai.Device.Config()
        #config.board.network.mtu = 9000
        config.board.network.xlinkTcpNoDelay = False
        with dai.Device(config, self.cam) as self.device:
            self.device.startPipeline(self.pipeline)


        # with dai.Device(self.pipeline,self.cam) as self.device:
            # self.device.setLogLevel(dai.LogLevel.DEBUG)
            # self.device.setLogOutputLevel(dai.LogLevel.DEBUG)
        ## Start the camera stream

            segmentation_queue = self.device.getOutputQueue("seg out",10,False)
            depth_queue = self.device.getOutputQueue("depth",1,False)
            left_queue = self.device.getOutputQueue(name="left", maxSize=2, blocking=False)
            right_queue = self.device.getOutputQueue("right",1,False)
            rgb_queue = self.device.getOutputQueue("rgb",1,False)
            self.qControl1 = self.device.getInputQueue(name="controlr")  
            self.qControl2 = self.device.getInputQueue(name="controlm")

            self.iso, self.ss = 6, 5
            self.miso, self.mss = 0, 5
            self.cor_grp = queue.Queue(nQ)
            self.mono_grp = queue.Queue(nQ)
            self.diffs = np.array([])
            self.diffs2 = np.array([])
            self.diffs3 = np.array([])
            self.diffs4 = np.array([])
            self.fulldiffs = np.array([])
            self.fetchdiffs = np.array([])
            stamp = time.time()

            for z in range(20):
                # c, r, d = rgb_queue.get(), right_queue.get(), depth_queue.get()
                c, r, d = rgb_queue.tryGet(), right_queue.get(), depth_queue.get()
                # r, d = right_queue.get(), depth_queue.get()
                # d = depth_queue.get()
            for z in range(10):
                c, r, d = rgb_queue.tryGet(), right_queue.get(), depth_queue.get()
                if c is not None:
                    rgb_out = c.getCvFrame()
                    self.add_frame(rgb_out, True)
                self.add_frame(r.getFrame(), False)
                if( time.time()-stamp > dT ):
                    self.iso, self.ss = self.adjust_exposure(self.iso, self.ss, True)
                    self.miso, self.mss = self.adjust_exposure(self.miso, self.mss, False)
                    stamp = time.time()

            rgb_out = None
            # right_out = None
            # depth_out = None
            while True:
                strt = dai.Clock.now()

                # rgb_in = rgb_queue.get()
                # rgb_out = rgb_in.getCvFrame()

                rgb_in = rgb_queue.tryGet()
                if rgb_in is not None:
                    rgb_out = rgb_in.getCvFrame()

                # fetchLatencyMs = (dai.Clock.now() - strt).total_seconds() * 1000
                # self.fetchdiffs = np.append(self.fetchdiffs, fetchLatencyMs)

                # last = dai.Clock.now()
                right_in = right_queue.get()
                right_out = right_in.getFrame()
                # LatencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                # self.diffs = np.append(self.diffs, LatencyMs)
                
                # last = dai.Clock.now()
                left_in = left_queue.get()
                # latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                # self.diffs2 = np.append(self.diffs2, latencyMs)
                # rgb_img = np.array(rgb_data.getCvFrame())

                # last = dai.Clock.now()
                depth_in = depth_queue.get()
                depth_out = depth_in.getFrame()
                # latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                # self.diffs3 = np.append(self.diffs3, latencyMs)
                # print("\n------- FETCHED IMAGE! ---------\n")
                # depth_img = np.array(depth_data.getCvFrame())
                
                last = dai.Clock.now()
                segmentation_labels = segmentation_queue.get()
                latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                self.diffs4 = np.append(self.diffs4, latencyMs)

                print("segmentation_labels", sys.getsizeof(segmentation_labels))
                

                # original model
                # seg_labels = (np.array(segmentation_labels.getFirstLayerFp16()).reshape(128,128)).astype(np.uint8)
                # larger model
                seg_labels = (np.array(segmentation_labels.getFirstLayerFp16()).reshape(64,64)).astype(np.uint8)
                # smaller model
                # seg_labels = (np.array(segmentation_labels.getFirstLayerInt32()).reshape(1024,1024)).astype(np.uint8)

                print("seg_labels", sys.getsizeof(segmentation_labels.getFirstLayerFp16()))

                if rgb_out is not None:
                    self.add_frame(rgb_out, True)
                self.add_frame(right_out, False)

                if( time.time()-stamp > dT ):
                    self.iso, self.ss = self.adjust_exposure(self.iso, self.ss, True)
                    self.miso, self.mss = self.adjust_exposure(self.miso, self.mss, False)
                    stamp = time.time()

                
                # print("\n------- AE ADJUSTED! ---------\n")

                if self.test_flag:
                    print("\n------- TAKING IMAGE! ---------\n")
                    # last = dai.Clock.now()
                    # Preparing PM3DCameraData
                    time_stamp = rospy.Time.now()
                    header = Header()
                    header.stamp = time_stamp
                    self.camera_data_msg.camera_name = self.camera_name
                    self.camera_data_msg.header = header
                    self.camera_data_msg.rgb_data = self.bridge.cv2_to_imgmsg(rgb_out,"bgr8") if rgb_out is not None else None
                    self.camera_data_msg.depth_map = self.bridge.cv2_to_imgmsg(depth_out,"mono16") if depth_out is not None else None
                    self.camera_data_msg.segmentation_labels = self.bridge.cv2_to_imgmsg(seg_labels,"mono8")
                    # print("\n------- CONVERTED IMAGE! ---------\n")

                    # latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                    # self.diffs = np.append(self.diffs, latencyMs)

                    #self.camera_data_msg.rgb_dims = rgb_out.shape 
                    #self.camera_data_msg.depth_map_dims = depth_out.shape 
                    #self.camera_data_msg.segmentation_label_dims = right_out.shape
                    #self.camera_data_msg.height_map_dims = depth_out.shape 

                    self.camera_data_msg.camera_id = self.camera_id
                    self.cam_location_req.gpscoords = [self.gps_data.latitude,self.gps_data.longitude]
                    self.cam_location_req.gpsheading = self.gps_data.gps_heading
                    self.cam_location_req.cameraid = self.camera_id

                    cam_location_response = self.cam_location(self.cam_location_req)

                    self.camera_data_msg.latitude = cam_location_response.newgpscoords[0]
                    self.camera_data_msg.longitude = cam_location_response.newgpscoords[1]
                    self.camera_data_msg.gps_heading = self.gps_data.gps_heading

                    # last = dai.Clock.now()
                    self.pub.publish(self.camera_data_msg)
                    if rgb_out is not None: self.image_pub.publish(self.camera_data_msg.rgb_data)
                    # print("\n------- PUBLISHED IMAGE! ---------\n")
                    # latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                    # self.diffs2 = np.append(self.diffs2, latencyMs)
                    self.test_flag = False

                    # rospy.loginfo(f"{self.camera_data_msg}")
                    # cv2.imshow(f"{self.node_name}",rgb_img)
                    # last = dai.Clock.now()
                    #self.__image_save(rgb_out,self.__rgbpath,self.camera_id,time_stamp)
                    #self.__image_save(depth_out,self.__depthpath,self.camera_id,time_stamp)
                    #self.__image_save(right_out,self.__segmentationpath,self.camera_id,time_stamp)
                    
                    # latencyMs = (dai.Clock.now() - last).total_seconds() * 1000
                    # self.diffs3 = np.append(self.diffs3, latencyMs)

                    # print("\n------- TOOK IMAGE! ---------\n")
                
                latencyMs = (dai.Clock.now() - strt).total_seconds() * 1000
                self.fulldiffs = np.append(self.fulldiffs, latencyMs)


                if cv2.waitKey(1) == ord('q') or (self.shutdown_flag == True):
                    # shutdown camera pipeline
                    break
            # print('\nRGB Fetch:\n', self.fetchdiffs, '\n\n')
            # print('\nRight Fetch:\n', self.diffs, '\n\n')
            # print('\nLeft Fetch:\n', self.diffs2, '\n\n')
            # print('\nDepth Fetch:\n', self.diffs3, '\n\n')
            print('\nSeg Fetch:\n', self.diffs4, '\n\n')
            print('\nFull Loop:\n', self.fulldiffs, '\n\n')
            
    def setisoss(self, fi, fs, col):
        ctr1 = dai.CameraControl()
        ctr1.setManualExposure(SS[fs], ISO[fi])
        if col:
            self.qControl1.send(ctr1)
        else:
            self.qControl2.send(ctr1)

    def add_frame(self, img, col):
        if col:
            im = cv2.resize(img, (406,304), interpolation = cv2.INTER_AREA)
            Y = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            hist = cv2.calcHist([Y],[0],None,[32],[0,256])
            if( self.cor_grp.qsize()>=nQ ): self.cor_grp.get(0)
            self.cor_grp.put(hist)
        else:
            Y = cv2.resize(img, (320,180), interpolation = cv2.INTER_AREA)
            hist = cv2.calcHist([Y],[0],None,[32],[0,256])
            if( self.mono_grp.qsize()>=nQ ): self.mono_grp.get(0)
            self.mono_grp.put(hist)

    def adjust_exposure(self, i, s, col):
        frm = []
        if col:
            for p in range(self.cor_grp.qsize()):    frm.append(w[p]*self.cor_grp.queue[p])
        else:
            for p in range(self.mono_grp.qsize()):    frm.append(w[p]*self.mono_grp.queue[p])
        arr = np.array(frm)
        
        summ = arr.sum(axis=0)
        peak = np.max(summ)
        pos = np.where(summ == peak)[0][0]

        if pos not in range(B, T): # 104 to 128 range by default
            # increase iso or ss
            if( pos < B):
                if s == len(SS)-1:
                    i = min(i+1, nISO)
                else:
                    s = min(s+1, 5)
            else:
                if i == 0:
                    s = max(0, s-1)
                else:
                    i = max(0, i-1)
        if col: self.setisoss(i, s, True)
        else:   self.setisoss(i, s, False)
        return i, s

    def __image_save(self,data,__path,cameraid, timestamp):

        image_name = "image_"+str(cameraid)+"_"+str(timestamp)+".png"
        cv2.imwrite(__path+image_name,data)

    def run_threads(self):
        t1 = threading.Thread(target=self.enable_camera)
        t2 = threading.Thread(target=self.run_camera_trigger_subscriber)

        # starting camera stream
        t1.start()
        # starting camera trigger subscriber
        t2.start()

        t1.join()
        t2.join()

    def oakd_callback(self,data):

        self.gps_data = data
        self.test_flag = True
            
    def run_camera_trigger_subscriber(self):

        while not rospy.is_shutdown(): 
            rospy.Subscriber('camera_trigger',PM3DGPSData,callback=self.oakd_callback)
            rospy.spin()

if __name__ == "__main__":
    
    cmd_rec = sys.argv[1:]
    seg_pipeline = PM3DCameraDataPublisher(cmd_rec[0],cmd_rec[1],cmd_rec[2])
    # seg_pipeline.enable_camera()
    seg_pipeline.run_threads()