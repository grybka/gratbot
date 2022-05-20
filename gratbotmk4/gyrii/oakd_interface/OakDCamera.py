
import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter
import cv2
from oakd_interface.OakDElement import OakDElement

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class OakDCamera(OakDElement):
    def __init__(self,pipeline,preview_size,fps,preview_streamname="rgb"):
        self.preview_streamname=preview_streamname
        self.preview_size=preview_size
        self.fps=fps

        logger.info("Creating RGB Camera in pipeline with size [{}x{}] at {} fps".format(preview_size[0],preview_size[1],fps))
        camRgb = pipeline.createColorCamera()
        camRgb.setFps(fps)
        camRgb.setPreviewSize(preview_size[0], preview_size[1])
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        #setup the preview output stream
        if preview_streamname is not None:
            xoutRgb = pipeline.createXLinkOut()
            xoutRgb.setStreamName(self.preview_streamname)
            camRgb.preview.link(xoutRgb.input)
        self.camRgb=camRgb

    def build_queues(self,device):
        if self.preview_streamname is not None:
            self.previewQueue = device.getOutputQueue(name=self.preview_streamname, maxSize=4, blocking=False)

    def tryget(self,broker):
        if self.preview_streamname is not None:
            inPreview = self.previewQueue.tryGet()
            if inPreview is not None:
                frame = inPreview.getCvFrame()
                frame_message={"timestamp": time.time()}
                image_timestamp=inPreview.getTimestamp().total_seconds()
                #logger.debug("image at {}".format(image_timestamp))
                frame_message["image_timestamp"]=image_timestamp
                frame_message["image"]=frame
                frame_message["keys"]=["image"]
                sequenceNum=inPreview.getSequenceNum()
                logger.debug("Image squence num {}".format(sequenceNum))
                broker.publish(frame_message,frame_message["keys"])

class OakDManip(OakDElement):
    def __init__(self,pipeline,new_size,camRgb):
        #create a manipulation that resizes the camera preview
        #camRgb is a ColorCamera object
        #new_size is an array [new_x,new_y]
        #the presumption is that the aspect ratio can be squeezed
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(new_size[0], new_size[1])
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
        manip.initialConfig.setKeepAspectRatio(False)
        camRgb.preview.link(manip.inputImage)
        self.manip=manip
    #TODO this doesn't necessarily need a tryget, but you could imagine one

class OakDDepth(OakDElement):
    #create the depth cameras
    #create an output stream if given one
    def __init__(self,pipeline,streamname=None):
        #depth camera
        self.streamname=streamname
        logger.info("Creating Stereo Camera in pipeline")

        monoLeft = pipeline.createMonoCamera()
        monoRight = pipeline.createMonoCamera()
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo = pipeline.createStereoDepth()
        stereo.setConfidenceThreshold(255)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        if streamname is not None:
            depthout=pipeline.createXLinkOut()
            depthout.setStreamName(streamname)
            stereo.disparity.link(depthout.input)
        self.stereo=stereo

    def build_queues(self,device):
        if self.streamname is not None:
            self.depthQueue = device.getOutputQueue(name=self.streamname, maxSize=4,blocking=False)

    def tryget(self,broker):
        #get depth from the depth queu
        #return the depth image if possible or none
        if self.streamname is not None:
            inDepth = self.depthQueue.tryGet()
            if inDepth is not None:
                frame=inDepth.getFrame()
                frame_message={"timestamp": time.time()}
                image_timestamp=inDepth.getTimestamp().total_seconds()
                frame_message["image_timestamp"]=image_timestamp
                frame_message["depth_image"]=cv2.resize(frame,(100,160) )
                frame_message["keys"]=["depth"]
                broker.publish(frame_message,frame_message["keys"])
