
from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
import logging,time
import numpy as np
import cv2

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class CameraDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display
        self.show_fps=True
        self.fps=0
        self.fps_count=11
        self.fps_count_reset=10
        self.fps_start_time=0

        self.mode="show_detections"
        #self.mode="show_tracks"
        super().__init__(broker)

        self.last_tracks_message={"tracks": []}
        self.last_detections_message={"detections": []}
        self.last_image_message={}

    def get_keys(self):
        return ["image","tracks"]

    def get_name(self):
        return "CameraDisplayGyrus"

    def update_fps_and_put_text(self,frame):
        self.fps_count+=1
        if self.fps_count>=self.fps_count_reset:
            self.fps=self.fps_count_reset/(time.time()-self.fps_start_time)
            self.fps_start_time=time.time()
            self.fps_count=0
        color = (255, 0, 0)
        cv2.putText(frame, "NN fps: {:.2f}".format(self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

    def update_display(self):
        if "image" not in self.last_image_message:
            return
        frame=np.copy(self.last_image_message["image"])
        if self.mode=="show_detections":
            color = (255, 0, 0)
            height = frame.shape[0]
            width  = frame.shape[1]
            for d in self.last_detections_message["detections"]:
                try:
                    x1 = int(d["bbox_array"][0] * width)
                    x2 = int(d["bbox_array"][1] * width)
                    y1 = int(d["bbox_array"][2] * height)
                    y2 = int(d["bbox_array"][3] * height)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                    cv2.putText(frame, str(d["label"]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                except:
                    logger.warning("failed to display deteciton {}".format(d))
                if "confidence" in d:
                    cv2.putText(frame, "{:.2f}".format(d["confidence"]*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                if "spatial_array" in d:
                    cv2.putText(frame, f"X: {int(d['spatial_array'][0])} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Y: {int(d['spatial_array'][1])} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Z: {int(d['spatial_array'][2])} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        elif self.mode=="show_tracks":
            color = (255, 0, 0)
            for t in self.last_tracks_message["tracks"]:
                x1 = int(t["bbox_array"][0] )
                x2 = int(t["bbox_array"][1] )
                y1 = int(t["bbox_array"][2] )
                y2 = int(t["bbox_array"][3] )
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                #cv2.putText(frame, str(id_to_name(t["id"])), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{} ({})".format(id_to_name(t["id"]),t["label"]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f} {:.2f} Dist: {:.2f}m".format(t["center"][0],t["center"][1],t["center"][2]), (x1+10,y1+35),cv2.FONT_HERSHEY_TRIPLEX,0.5,255)
        if self.show_fps==True:
            self.update_fps_and_put_text(frame)
        self.display.update_image("camera",frame)

    def read_message(self,message):
        if "image" in message:
            self.last_image_message=message
            self.update_display()
        if "detections" in message:
            self.last_detections_message=message
        if "tracks" in message:
            self.last_tracks_message=message
