#Functions to use MobilenetV2 models on oak d

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

#Make a MobilenetV2 spatial detection network
#name,shaves is the name of the blob
#camera is the source of images
#stereo is the source of stereo information
#streamname is the name of the output stream for detections
class OakDMobileNetDetections(OakDElement):
    def __init__(self,pipeline,model_name,shaves,camera,stereo,streamname,model_labels):
        self.streamname=streamname
        logger.info("Creating MobilenetV2 Detections: {}".format(self.streamname))
        self.model_labels=model_labels
        spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
        spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves)))
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)
        camera.link(spatialDetectionNetwork.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        if self.streamname is not None:
            xoutNN = pipeline.createXLinkOut()
            xoutNN.setStreamName(streamname)
            spatialDetectionNetwork.out.link(xoutNN.input)
        self.spatialDetectionNetwork=spatialDetectionNetwork

    def build_queues(self,device):
        if self.streamname is not None:
            self.detectionNNQueue = device.getOutputQueue(name=self.streamname, maxSize=4, blocking=False)

    def tryget(self,broker):
        if self.streamname is None:
            return #no detections to get
        inDet = self.detectionNNQueue.tryGet()
        if inDet is not None:
            device_timestamp=inDet.getTimestamp().total_seconds()
            detection_message=[]
            for detection in inDet.detections:
                det_item={}
                bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                det_item["bbox_array"]=bbox_array
                det_item["spatial_array"]=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
                det_item["label"] = self.model_labels[detection.label]
                det_item["confidence"] = detection.confidence
                detection_message.append(det_item)
            #Send a detections message even if its empty
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]=self.streamname
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections",self.streamname])
