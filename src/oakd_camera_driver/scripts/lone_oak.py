#!/usr/bin/env python3
"""
@author: MathewAaron
"""
import time
import threading
import depthai as dai
import numpy as np
import cv2
import glob
import os, sys
import queue
os.chdir(os.path.dirname(os.path.abspath(__file__)))

nISO = 19
ISO = np.logspace(1.25, 2, num=nISO+1, endpoint=True, base=40, dtype=int)
SS = [313, 400, 500, 625, 800, 1000]
T = 15
B = 12
dT = 0    #1.99
nQ = 1




class PM3DCameraDataPublisher():
    """
    This class is a driver for streaming the PM3DCamera Data driver for oakd cameras.
    This function returns RGBD and segmentation labels.
    """
    def __init__(self,ip,node_name:str,cameraid:int):
        
        # Temporary camera id
        self.camera_id = int(cameraid)
        self.pipeline = dai.Pipeline()
      
        print("Creating color camera node...")
        self.RGB_Node = self.pipeline.createColorCamera()
        self.RGB_Node.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        self.RGB_Node.setBoardSocket(dai.CameraBoardSocket.RGB)
        # self.RGB_Node.setSharpness(0)     # range: 0..4, default: 1    
        # self.RGB_Node.setLumaDenoise(0)   # range: 0..4, default: 1    
        # self.RGB_Node.setChromaDenoise(4) # range: 0..4, default: 1

        # modifying isp frame and then feeding it to encoder
        self.manip = self.pipeline.create(dai.node.ImageManip)
        self.manip.initialConfig.setCropRect(0.006, 0, 1, 1)
        # self.manip.setNumFramesPool(2)
        self.manip.setMaxOutputFrameSize(18385920)
        self.manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        self.RGB_Node.isp.link(self.manip.inputImage)
        self.manip.inputImage.setBlocking(False)

        self.videoEnc = self.pipeline.create(dai.node.VideoEncoder)
        self.videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
        self.manip.out.link(self.videoEnc.input)
        self.videoEnc.input.setBlocking(False)

        self.RGB_Out=self.pipeline.create(dai.node.XLinkOut)
        self.RGB_Out.setStreamName("rgb")
        self.videoEnc.bitstream.link(self.RGB_Out.input)
        print("Done")

        print("Creating depth channel...")
        self.monoL = self.pipeline.create(dai.node.MonoCamera)
        self.monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)

        self.monoR = self.pipeline.create(dai.node.MonoCamera)
        self.monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.depth.setLeftRightCheck(True)
        self.depth.setExtendedDisparity(False)
        self.depth.setSubpixel(False)
        self.monoL.out.link(self.depth.left)
        self.monoR.out.link(self.depth.right)
        print("Done")
        
        print("Seting up right mono camera...")
        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight.setStreamName("right")
        self.monoR.out.link(self.xoutRight.input)

        self.depth_out = self.pipeline.create(dai.node.XLinkOut)
        self.depth_out.setStreamName("depth")
        self.depth.depth.link(self.depth_out.input)
        print("Done")

        self.RGB_Node.setFps(10)
        self.monoR.setFps(10)
        self.monoL.setFps(10)

        self.RGB_Node.setIsp3aFps(1)
        self.monoR.setIsp3aFps(1)
        self.monoL.setIsp3aFps(1)

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

        self.shutdown_flag = False
        self.cam = dai.DeviceInfo(ip)
        
    def __shutdown(self):
        self.shutdown_flag = True

    def enable_camera(self):
        with dai.Device(self.pipeline,self.cam) as self.device:
            self.device.setLogLevel(dai.LogLevel.DEBUG)
            self.device.setLogOutputLevel(dai.LogLevel.DEBUG)
            ## Start the camera stream
            depth_queue = self.device.getOutputQueue("depth",1,False)
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
            stamp = time.time()

            for z in range(10):
                c, r, d = rgb_queue.get(), right_queue.get(), depth_queue.get()
            for z in range(10):
                c, r, d = rgb_queue.get(), right_queue.get(), depth_queue.get()
                img = cv2.imdecode(c.getData(), cv2.IMREAD_COLOR)
                self.add_frame(img, True)
                self.add_frame(r.getFrame(), False)
                self.iso, self.ss = self.adjust_exposure(self.iso, self.ss, True)
                self.miso, self.mss = self.adjust_exposure(self.miso, self.mss, False)

            while True:
                right_in = right_queue.get()
                rgb_in = rgb_queue.get()
                depth_in = depth_queue.get()

                decode = rgb_in.getData()
                time.sleep(0.07)
                #rgb_out = cv2.imdecode(decode, cv2.IMREAD_COLOR)
                #right_out = right_in.getFrame()

                #time.sleep(0.175)

                #self.add_frame(rgb_out, True)
                #self.add_frame(right_out, False)
                

                #if( time.time()-stamp > dT ):
                #    self.iso, self.ss = self.adjust_exposure(self.iso, self.ss, True)
                #    self.miso, self.mss = self.adjust_exposure(self.miso, self.mss, False)
                #    stamp = time.time()

                
                print("\n------- AE ADJUSTED! ---------\n")

            print('\n\nFlatten:\n\n', self.diffs, '\n\n')
            print('\nPublish:\n', self.diffs2, '\n\n')
            print('\nSaving:\n', self.diffs3, '\n\n')
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
            for p in range(self.cor_grp.qsize()):    frm.append(self.cor_grp.queue[p])
        else:
            for p in range(self.mono_grp.qsize()):    frm.append(self.mono_grp.queue[p])
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
        # starting camera stream
        t1.start()
        t1.join()


if __name__ == "__main__":
    
    cmd_rec = sys.argv[1:]
    seg_pipeline = PM3DCameraDataPublisher(cmd_rec[0],cmd_rec[1],cmd_rec[2])
    # seg_pipeline.enable_camera()
    seg_pipeline.run_threads()