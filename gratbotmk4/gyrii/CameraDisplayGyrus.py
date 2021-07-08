
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

        self.show_detections=True
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
                if self.fps_count>=self.fps_count_reset:
                    self.fps=self.fps_count_reset/(time.time()-self.fps_start_time)
                    self.fps_start_time=time.time()
                    self.fps_count=0
                color = (255, 0, 0)
                cv2.putText(frame, "NN fps: {:.2f}".format(self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            if self.show_detections==True and "detections" in message:
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

            self.display.update_image("camera",message["image"])
