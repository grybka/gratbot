
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
from underpinnings.OakDItems import *

logger=logging.getLogger(__name__)

logger.setLevel(logging.DEBUG)
# Tiny yolo v3/4 label texts
labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

class OakDGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.oak_comm_thread=None
        #self.model="person-detection-retail-0013"
        self.models=[]
        self.models.append({"modelname": "face-detection-0200",
                       "streamname": "face_detections",
                       "labels": ["face"]})
        self.models.append({"modelname": "person-detection-0200",
                       "streamname": "person_detections",
                       "labels": ["person"]})
        self.local_rotation=np.zeros(3)
        self.last_gyro_Ts=0
        self.broker=broker
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
        return "OakDGyrusPeople"

    def read_message(self,message):
        self.output_queue.put(object_to_json)

    def _oak_comm_thread_loop(self):
        self.init_oakd()
        with dai.Device(self.pipeline) as device:
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            #previewQueue = device.getOutputQueue(name="mrgb", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4,blocking=False)
            detectionNNQueue=device.getOutputQueue(name="detections",maxSize=4,blocking=False)
            #for model in self.models:
            #    model["queue"] = device.getOutputQueue(name=model["streamname"], maxSize=4, blocking=False)
                #model["queue_passthru"] = device.getOutputQueue(name=model["streamname"]+"_passthru", maxSize=4, blocking=False)
            logging.debug("OakD created and queue's gotten")
            last_frame=None
            while not self.should_quit:
                tryget_imudata(imuQueue,self.broker)
                tryget_depth(depthQueue,self.broker)
                image_timestamp,frame=tryget_image(previewQueue,self.broker)
                if frame is not None:
                    last_frame=frame

                tryget_nndetections(detectionNNQueue,self.broker,labelMap)
                #MOODEL THNIGLABL
                #for model in self.models:
                    #tryget_nndetections(model["queue"],model["queue_passthru"],self.broker,last_frame,model["labels"])
                #    tryget_nndetections_nopassthru(model["queue"],self.broker,model["labels"],model["streamname"])
        logging.debug("Exiting OakD thread")

    def init_oakd(self):
        self.pipeline = dai.Pipeline()
        #self.pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_4)

        #setup accelerometer/magnetometere
        logger.info("Creating IMU in pipeline")
        create_imu(self.pipeline,"imu")

        #setup camera
        logger.info("Creating RGB Camera in pipeline")
        camRgb = self.pipeline.createColorCamera()
        camRgb.setFps(15)
        #camRgb.setPreviewSize(640, 360)
        camRgb.setPreviewSize(640, 480)
        #camRgb.setPreviewSize(320, 240)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        #camRgb.video.link(xoutRgb.input)
        #camRgb.video.link(xoutRgb.input)
        camRgb.preview.link(xoutRgb.input)

# Create manip
        NN_WIDTH=416
        NN_HEIGHT=416
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(NN_WIDTH, NN_HEIGHT)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
        manip.initialConfig.setKeepAspectRatio(False)
        camRgb.preview.link(manip.inputImage)

#stereo
        stereo=create_depth(self.pipeline,"depth")

#yolo
        spatialDetectionNetwork=create_yolo(self.pipeline,stereo)
        manip.out.link(spatialDetectionNetwork.input)
