
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
class OakDTracker(OakDElement):
    def __init__(self,pipeline,camRgb,stereo,spatialDetectionNetwork):
        self.streamname="tracklets"
        trackerOut = pipeline.create(dai.node.XLinkOut)

        trackerOut.setStreamName(self.streamname)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        objectTracker.setDetectionLabelsToTrack([0])  # TODO fix
        # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
        objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
        objectTracker.out.link(trackerOut.input)
        camRgb.video.link(objectTracker.inputTrackerFrame)
        objectTracker.inputTrackerFrame.setBlocking(False)
        objectTracker.inputTrackerFrame.setQueueSize(2)
        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        self.objectTracker=objectTracker

    def build_queues(self,device):
        self.trackletQueue = device.getOutputQueue(name=self.streamname, maxSize=4, blocking=False)


    def tryget(self,broker):
        track = self.trackletQueue.tryGet()
        if track is not None:
            device_timestamp=track.getTimestamp().total_seconds()
            detection_message=[]
            for t in trackletsData:
                det_item={}
                bbox_array=[roi.topLeft().x,roi.bottomRight().x,roi.bottomRight().y,roi.topLeft().y]
                det_item["bbox_array"]=bbox_array
                det_item["spatial_array"]=[t.spatialCoordinates.x,t.spatialCoordinates.y,t.spatialCoordinates.z]
                det_item["label"] = "{}".format(t.label)
                det_item["confidence"] = 1
                detection_message.append(det_item)
            if len(detection_message)!=0:
                frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
                frame_message["detection_name"]=self.streamname
                frame_message["detections"]=detection_message
                broker.publish(frame_message,["detections",self.streamname])
