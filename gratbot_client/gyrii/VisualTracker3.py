
from Gyrus import ThreadedGyrus
import time
import numpy as np
from scipy.linalg import block_diag
import torch
import cv2 as cv
import uuid
from gyrii.underpinnings.id_to_name import id_to_name
from gyrii.underpinnings.GratbotLogger import gprint
from gyrii.underpinnings.Vodometer import Vodometer,VodometerException
from gyrii.underpinnings.BayesianArray import BayesianArray
#from gyrii.underpinnings.ConvolutionalVisualOdometer import ConvolutionalVisualOdometer
from scipy.optimize import linear_sum_assignment
from filterpy.common import kinematic_kf,kinematic_state_transition,Q_discrete_white_noise

class VisualTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,display,show_detections=True):
        self.show_detections=show_detections
        self.display=display
        self.clear_frames_before=0

        self.objects_to_track=["chair","person","sports ball","stop sign"]

        #Odometer
        self.vodometer=Vodometer()

        #Tagging info
        self.tagger_model= torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.tagger_classes=self.tagger_model.module.names if hasattr(self.tagger_model,'module') else self.tagger_model.names
        self.tagged_objects=[] #where i keep the last tag results



        self.n_iter_report=-1
        self.on_iter=0

        self.tagging_time_sum=0
        self.vodometer_time_sum=0
        super().__init__(broker)

    def get_keys(self):
        return [ "camera_frame","drive/motors_active","latest_pose" ]

    def get_name(self):
        return "VisualTrackerGyrus"

    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                return
            retval=self.update_with_frame(message["camera_frame"],message["timestamp"])
            if self.show_detections and retval:
                self.draw_bboxes(message["camera_frame"])

    def update_with_frame(self,frame,timestamp):
        start_time=time.time()
        try:
            x_offset,y_offset,x_offset_stdev,y_offset_stdev=self.vodometer.get_offset_since_last(frame)
            self.broker.publish({"video_offset": [x_offset,y_offset,x_offset_stdev,y_offset_stdev],"timestamp": timestamp,"notes": "visual"},"video_offset")
        except VodometerException as e:
            #gprint("Vodometer exception {}".format(e.message))
            pass
        self.vodometer_time_sum+=time.time()-start_time

        self.tagged_objects=self.tag_objects(frame)
        self.broker.publish({"tagged_objects": self.tagged_objects,"timestamp": timestamp,"notes": "visual"},"tagged_objects")

        self.on_iter+=1
        if self.n_iter_report>0 and self.on_iter>self.n_iter_report:
            tagging_time=self.tagging_time_sum/self.on_iter
            gprint("Average Tagging Time {} ms".format(tagging_time*1000))
            vodometer_time=self.vodometer_time_sum/self.on_iter
            gprint("Average Vodometer Time {} ms".format(vodometer_time*1000))
            self.tagging_time_sum=0
            self.vodometer_time_sum=0
            self.on_iter=0
        return True

    def tag_objects(self,frame):
        start_time=time.time()
        imgs=[frame]
        results = self.tagger_model(imgs)
        video_objects=[]
        for r in results.xyxy[0]:
            video_objects.append({
                "confidence": r[4].item(),
                "label": self.tagger_classes[int(r[5].item())],
                "startx": r[0].item(),
                "starty": r[1].item(),
                "endx": r[2].item(),
                "endy": r[3].item()
            })
        self.tagging_time_sum+=time.time()-start_time
        return video_objects


    def draw_bboxes(self,video_frame):
        #gprint("draw bboxes")
        if video_frame is None:
            gprint("bboxes: Gave me a none video frame")
            return

        ret_frame=video_frame.copy()
        if True: #mode is just tags
            for obj in self.tagged_objects:
                sx,ex,sy,ey=obj["startx"],obj["endx"],obj["starty"],obj["endy"]
                cv.rectangle(ret_frame,(int(sx),int(sy)),(int(ex),int(ey)),(0,255,0),2)
                text = "{} {}".format("Tagged ",obj["label"])
                Y = int(sy - 10 if sy - 10 > 10 else sy + 10)
                cv.putText(ret_frame, text, (int(sx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
        self.display.update_image("visual_tracker",ret_frame)
