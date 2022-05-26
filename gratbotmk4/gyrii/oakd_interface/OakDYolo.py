
import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter
from oakd_interface.OakDElement import OakDElement

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#Lesson spatialdetectionnetwork will give me the wrong values when I'm squeezing or letterboxing
#I have to do it on my own

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

class OakDYoloDetections(OakDElement):
    def __init__(self,pipeline,model_name,shaves,camera,streamname,model_labels,confidence_threshold=0.3):
        logger.info("Creating Detection Network")
        #TODO I should have info about how to map my bounding boxes here
        #right now it's assuming that I've squeezed
        self.ok_labels=[labelMap.index("person"),labelMap.index("sports ball"),labelMap.index("stop sign")]
        #self.ok_labels=[]
        self.streamname=streamname
        #spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
        spatialDetectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves,zoo_type="depthai")))
        #spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(confidence_threshold)
        spatialDetectionNetwork.input.setBlocking(False)

        # Yolo specific parameters
        spatialDetectionNetwork.setNumClasses(80)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
        spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
        spatialDetectionNetwork.setIouThreshold(0.5)

        xoutNN = pipeline.createXLinkOut()
        xoutNN.setStreamName(streamname)
        spatialDetectionNetwork.out.link(xoutNN.input)
        camera.link(spatialDetectionNetwork.input)
        self.detectionNetwork=spatialDetectionNetwork
        self.model_labels=labelMap

    def build_queues(self,device):
        self.detectionNNQueue = device.getOutputQueue(name=self.streamname, maxSize=4, blocking=False)

    def tryget(self,broker):
        inDet = self.detectionNNQueue.tryGet()
        if inDet is not None:
            device_timestamp=inDet.getTimestamp().total_seconds()
            detection_message=[]
            for detection in inDet.detections:
                if len(self.ok_labels)!=0 and detection.label not in self.ok_labels:
                    continue
                det_item={}
                bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                det_item["bbox_array"]=bbox_array
                det_item["label"] = self.model_labels[detection.label]
                det_item["confidence"] = detection.confidence
                detection_message.append(det_item)
            #Send a detections message even if its empty
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]=self.streamname
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections",self.streamname])

class OakDYoloDetectionsSpatial(OakDElement):
    def __init__(self,pipeline,model_name,shaves,camera,stereo,streamname,confidence_threshold=0.3,spatial_x_scale=1.0):
        logger.info("Creating Spatial Detection Network")
        self.ok_labels=[labelMap.index("person"),labelMap.index("sports ball")]
        self.spatial_x_scale=spatial_x_scale #for cases where I've squeezed the resolution
        self.streamname=streamname
        #spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves,zoo_type="depthai")))
        #spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(confidence_threshold)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        spatialDetectionNetwork.setNumClasses(80)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
        spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
        spatialDetectionNetwork.setIouThreshold(0.5)

        xoutNN = pipeline.createXLinkOut()
        xoutNN.setStreamName(streamname)
        spatialDetectionNetwork.out.link(xoutNN.input)
        camera.link(spatialDetectionNetwork.input)
        stereo.link(spatialDetectionNetwork.inputDepth)
        self.spatialDetectionNetwork=spatialDetectionNetwork
        self.model_labels=labelMap

    def build_queues(self,device):
        self.detectionNNQueue = device.getOutputQueue(name=self.streamname, maxSize=4, blocking=False)

    def tryget(self,broker):
        inDet = self.detectionNNQueue.tryGet()
        if inDet is not None:
            device_timestamp=inDet.getTimestamp().total_seconds()
            detection_message=[]
            for detection in inDet.detections:
                if detection.label not in self.ok_labels:
                    continue
                det_item={}
                bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                det_item["bbox_array"]=bbox_array
                det_item["spatial_array"]=[self.spatial_x_scale*detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
                det_item["label"] = self.model_labels[detection.label]
                det_item["confidence"] = detection.confidence
                detection_message.append(det_item)
            #Send a detections message even if its empty
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]=self.streamname
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections",self.streamname])
