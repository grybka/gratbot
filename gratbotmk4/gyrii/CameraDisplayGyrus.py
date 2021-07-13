
from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
import logging,time
import numpy as np
import cv2

class CameraDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display
        self.show_fps=True
        self.fps=0
        self.fps_count=11
        self.fps_count_reset=10
        self.fps_start_time=0

        #self.mode="show_detections"
        #self.mode="show_tracks"
        self.mode="just camera"
        super().__init__(broker)

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

    def read_message(self,message):
        if "image" in message:
            frame=np.copy(message["image"])
            if self.mode=="show_detections":
                if "detections" in message:
                    color = (255, 0, 0)
                    height = frame.shape[0]
                    width  = frame.shape[1]
                    for d in message["detections"]:
                        x1 = int(d["bbox_array"][0] * width)
                        x2 = int(d["bbox_array"][1] * width)
                        y1 = int(d["bbox_array"][2] * height)
                        y2 = int(d["bbox_array"][3] * height)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                        cv2.putText(frame, str(d["label"]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, "{:.2f}".format(d["confidence"]*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"X: {int(d['spatial_array'][0])} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"Y: {int(d['spatial_array'][1])} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"Z: {int(d['spatial_array'][2])} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    if self.show_fps==True:
                        self.update_fps_and_put_text(frame)
                    self.display.update_image("camera",frame)
            elif self.mode=="show_tracks":
                if "tracks" in message:
                    color = (255, 0, 0)
                    for t in message["tracks"]:
                        x1 = int(t["bbox_array"][0] )
                        x2 = int(t["bbox_array"][1] )
                        y1 = int(t["bbox_array"][2] )
                        y2 = int(t["bbox_array"][3] )
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                        cv2.putText(frame, str(id_to_name(t["id"])), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    if self.show_fps==True:
                        self.update_fps_and_put_text(frame)
                    self.display.update_image("camera",frame)
#                else:
                    #logging.warning("not showing image with keys {}".format(message["keys"]))
            else:
                if self.show_fps==True:
                    self.update_fps_and_put_text(frame)
                self.display.update_image("camera",frame)
