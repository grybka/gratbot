from Gyrus import ThreadedGyrus
import time
import numpy as np
import logging
import torch
import cv2 as cv

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
#logger.setLevel(logging.INFO)


class ObjectTaggerGyrus(ThreadedGyrus):
    def __init__(self,broker,detection_name="detections_software"):
        super().__init__(broker)
        self.tagger_model= torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.tagger_classes=self.tagger_model.module.names if hasattr(self.tagger_model,'module') else self.tagger_model.names

        self.last_image_timestamp=0
        self.check_every=0.2 #5 fps

    def get_keys(self):
        return ["image","rotation_vector"]

    def get_name(self):
        return "ObjectTaggerGyrus"

    def tag_objects(self,frame):
        start_time=time.time()
        imgs=[frame]
        results = self.tagger_model(imgs)
        video_objects=[]
        height=frame.shape[0]
        width=frame.shape[1]
        for r in results.xyxy[0]:
            det_item={}
            sx,sy,ex,ey=r[0].item(),r[1].item(),r[2].item(),r[3].item()
            bbox_array=[ sx/width,ex/width,sy/height,ey/height ]
            det_item["bbox_array"]=bbox_array
            det_item["label"] = self.tagger_classes[int(r[5].item())]
            #logger.debug("detected {}".format(det_item["label"]))
            det_item["confidence"] = r[4].item()
            det_item["subimage"]=frame[int(sy):int(ey),int(sx):int(ex)]
            video_objects.append(det_item)
        return video_objects


    def read_message(self,message):
        if "image" in message:
            if message["image_timestamp"]<self.last_image_timestamp+self.check_every:
                return #skip this frame
            frame=cv.resize(message["image"],(320,200))
            start=time.time()
            video_objects=self.tag_objects(frame)
            delta_time=time.time()-start
            frame_message={"timestamp": time.time()}
            frame_message["detections"]=video_objects
            frame_message["detection_name"]=self.detection_name
            frame_message["image_timestamp"]=message["image_timestamp"]
            self.last_image_timestamp=frame_message["image_timestamp"]+delta_time
            self.broker.publish(frame_message,[self.detection_name])
