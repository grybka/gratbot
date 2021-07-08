
from Gyrus import ThreadedGyrus
import logging,time
import cv2

class CameraDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display
        self.show_fps=True
        self.fps=0
        self.fps_count=11
        self.fps_count_reset=10
        self.fps_start_time=0
        super().__init__(broker)

    def get_keys(self):
        return ["image"]

    def get_name(self):
        return "CameraDisplayGyrus"

    def read_message(self,message):
        if "image" in message:
            frame=message["image"]
            if self.show_fps==True:
                self.fps_count+=1
                if self.fps_count>self.fps_count_reset:
                    self.fps=self.fps_count_reset/(time.time()-self.fps_start_time)
                    self.fps_start_time=time.time()
                    self.fps_count=0
                color = (255, 0, 0)
                cv2.putText(frame, "NN fps: {:.2f}".format(self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            self.display.update_image("camera",message["image"])
