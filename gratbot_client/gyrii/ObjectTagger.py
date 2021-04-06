from Gyrus import ThreadedGyrus
import sys
sys.path.append('../yolov5_tool')
from yolov5_tool import yolov5_tool
import logging
import numpy as np
from gyrii.underpinnings.GratbotLogger import gprint

#let's include tracking here


class ObjectTaggerGyrus(ThreadedGyrus):
    def __init__(self,broker,display,show_detections=True):
        self.model_file="C:/Users/grybk/projects/yolov5/yolov5/weights/yolov5m.pt"
        self.video_width=640
        self.video_height=480
        self.yv5model=None
        self.yv5model=yolov5_tool()
        self.display=display
        self.clear_frames_before=0
        self.show_detections=show_detections
        super().__init__(broker)


    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "ObjectTaggerGyrus"

    def tag_objects(self,video_frame):
        if self.yv5model.initialized==False:
            logging.info("initializing model")
            self.yv5model.initialize(self.model_file)
            logging.info("initialized")
        #logging.info("detecting")
        try:
            video_objects=self.yv5model.detect(video_frame)
        except Exception as e:
            print("video frame is {}".format(video_frame))
            print("Exception: {}".format(e))
            print("Unexpected error:", sys.exc_info()[0])
        #logging.info("detected")
        return video_objects

    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                return
            video_objects=self.tag_objects(message["camera_frame"])
            #gprint("number of video objects: {}".format(len(video_objects)))
            mymessage={"timestamp":message["timestamp"],"video_object_tags": video_objects}
            self.broker.publish(mymessage,"video_object_tags")
            if self.show_detections:
                self.display_tags(message["camera_frame"],video_objects)
            self.clear_frames_before=time.time()

    def display_tags(self,video_frame,video_objects):
        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        return_frame=video_frame.copy()
        self.yv5model.draw_object_bboxes(return_frame,video_objects)
        self.display.update_image("object_tagger",return_frame)
