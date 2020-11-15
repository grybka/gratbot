import sys
sys.path.append('../yolov5_tool')
from yolov5_tool import yolov5_tool
import logging
import numpy as np

class Theo_Chaser_Object_Tagger():
    def __init__(self):
        #self.model_file="C:/Users/grybk/projects/yolov5/yolov5/runs/exp28/weights/last.pt"
        self.model_file="C:/Users/grybk/projects/yolov5/yolov5/weights/yolov5m.pt"
        self.video_width=640
        self.video_height=480
        self.yv5model=None
        self.yv5model=yolov5_tool()

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

    def draw_bboxes(self,video_frame,video_objects=None):
        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        if video_objects==None:
            video_objects=self.tag_objects(video_frame)
        self.yv5model.draw_object_bboxes(video_frame,video_objects)
        return video_frame

    def get_obj_loc_width(self,face):
        #converts the start,stop notation into an array of points, center and wh
        centerx=(0.5*(face["startx"]+face["endx"])-self.video_width/2)/self.video_width
        centery=(0.5*(face["starty"]+face["endy"])-self.video_height/2)/self.video_height
        width=(-face["startx"]+face["endx"])/self.video_width
        height=(-face["starty"]+face["endy"])/self.video_height
        return np.array([centerx,centery]),np.array([width,height])
