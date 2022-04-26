#Functions to use MobilenetV2 models on oak d

import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#Make a MobilenetV2 spatial detection network
#name,shaves is the name of the blob
#camera is the source of images
#stereo is the source of stereo information
#streamname is the name of the output stream for detections
def init_oakdmobilenet(name,shaves,camera,stereo,streamname):
    spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
    spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves)))
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)
    camera.link(spatialDetectionNetwork.input)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)
    xoutNN = pipeline.createXLinkOut()
    xoutNN.setStreamName(streamname)
    spatialDetectionNetwork.out.link(xoutNN.input)
    return spatialDetectionNetwork


def tryget_oakdmobilenet_detections(detectionNNQueue,broker,model_labels,streamname):
    inDet = detectionNNQueue.tryGet()
    if inDet is not None:
        device_timestamp=inDet.getTimestamp().total_seconds()
        detection_message=[]
        sequenceNum=inDet.getSequenceNum()
        logger.debug("NN squence num {}".format(sequenceNum))
        for detection in inDet.detections:
            det_item={}
            bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
            det_item["bbox_array"]=bbox_array
            det_item["spatial_array"]=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
            det_item["label"] = model_labels[detection.label]
            det_item["confidence"] = detection.confidence
            det_item["subimage"] = 0
            detection_message.append(det_item)
        if len(detection_message)!=0:
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]=streamname
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections",streamname])
        return None
    else:
        return None
    ...
