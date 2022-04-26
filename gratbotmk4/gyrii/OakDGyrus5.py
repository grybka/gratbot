
from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time
import os
import numpy as np
import cv2
#import blobconverter
from pathlib import Path
import blobconverter

from oakd_interface.OakDIMU import OakDIMU
from oakd_interface.OakDCamera import OakDCamera,OakDDepth,OakDManip
from oakd_interface.OakDMobileNet import OakDMobileNetDetections
from oakd_interface.OakDYolo import OakDYoloDetections

class OakDGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.oak_comm_thread=None
        self.broker=broker
        self.preview_size=[320,240]
        self.fps=20
        self.elements=[]
        super().__init__(broker)

    def start_thread_called(self):
        logging.debug("starting OakD Comms Thread")
        self.oak_comm_thread = threading.Thread(target=self._oak_comm_thread_loop)
        self.oak_comm_thread.daemon = True
        self.oak_comm_thread.start()

    def join_called(self):
        if self.oak_comm_thread is not None:
            return self.oak_comm_thread.join()
        return None

    def get_keys(self):
        return [] #nothing yet

    def get_name(self):
        return "OakDGyrus5"

    def read_message(self,message):
        ... #TODO this could reload what models I use

    def init_oakd(self):
        pipeline = dai.Pipeline()
        camera=OakDCamera(pipeline,self.preview_size,self.fps,preview_streamname="rgb")
        stereo=OakDDepth(pipeline)
        #manip=OakDManip(pipeline,[256,256],camera.camRgb)
        manip=OakDManip(pipeline,[416,416],camera.camRgb)
        self.elements.append(OakDIMU(pipeline))
        self.elements.append(camera)
        self.elements.append(stereo)
        #self.elements.append(OakDMobileNetDetections(pipeline,"face-detection-0200",6,manip.manip.out,stereo.stereo,"face_detections",["face"]))
        self.elements.append(OakDYoloDetections(pipeline,"yolov4_tiny_coco_416x416",6,manip.manip.out,stereo.stereo,"detections",None))
        self.pipeline=pipeline

    def _oak_comm_thread_loop(self):
        self.init_oakd()
        with dai.Device(self.pipeline) as device:
            for element in self.elements:
                element.build_queues(device)
            while not self.should_quit:
                for element in self.elements:
                    element.tryget(self.broker)
        logging.debug("Exiting OakD thread")
