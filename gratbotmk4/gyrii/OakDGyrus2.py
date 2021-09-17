
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

#From the camera, generate
# - yolo detections with subimages
# - depth map
# - 640x480 full image

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

class OakDGyrusPeople(ThreadedGyrus):
    def __init__(self,broker):
        self.oak_comm_thread=None
        #self.model="person-detection-retail-0013"
        self.models=[ {"modelname": "person-detection-0200",
                       "streamname": "person_detections",
                       "labels": ["person"]},
                      {"modelname": "face-detection-0200",
                       "streamname": "face_detections",
                       "labels": ["face"]}]
        #self.model1="person-detection-0200"
        #self.model2="face-detection-0200"
        self.local_rotation=np.zeros(3)
        self.last_gyro_Ts=0
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
            depthQueue = device.getOutputQueue(name="depth", maxSize=4,blocking=False)
            for model in self.models:
                model["queue"] = device.getOutputQueue(name=model["streamname"], maxSize=4, blocking=False)
            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
                tryget_imudata(imuQueue,broker)
                tryget_depth(depthQueue,broker)
                image_timestamp,frame=tryget_image(previewQueue,broker)
                #MOODEL THNIGLABL
                tryget_nndetections(detectionNNQueue,broker,image_frame,model_labels)
        logging.debug("Exiting OakD thread")

    def init_oakd(self):
        self.pipeline = dai.Pipeline()
        self.pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_2)

        #setup accelerometer/magnetometere
        logger.info("Creating IMU in pipeline")
        create_imu(self.pipline,"imu")

        #setup camera
        logger.info("Creating RGB Camera in pipeline")
        camRgb = self.pipeline.createColorCamera()
        #camRgb.setFps(20)
        #camRgb.setPreviewSize(544, 320)
        camRgb.setPreviewSize(640, 480)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        manip = pipeline.createImageManip()
        manip.initialConfig.setResize(416, 416)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        camRgb.preview.link(manip.input)

        #stereo
        stereo=create_depth(self.pipeline,"depth"):

        #yolo
        yolo=create_yolo(self.pipline)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        manip.output.link(spatialDetectionNetwork.input)

        #depth camera
