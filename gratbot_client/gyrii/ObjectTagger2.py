import torch
import cv2 as cv
from Gyrus import ThreadedGyrus
from gyrii.underpinnings.GratbotLogger import gprint
import time


class ObjectTaggerGyrus(ThreadedGyrus):
    def __init__(self,broker,display,show_detections=True):
        self.clear_frames_before=0
        self.show_detections=show_detections
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.classes=self.model.module.names if hasattr(self.model,'module') else self.model.names
        self.display=display
        super().__init__(broker)

        #profiling
        self.n_sample=10
        self.time_sum=0
        self.sample_sum=0

    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "ObjectTaggerGyrus"

    def update(self,frame,timestamp):
        #dim=(320,240)
        #resized_frame=cv.resize(frame,dim)
        imgs=[frame]
        results = self.model(imgs)
        # xmin,ymin,xmax,yma,confidence,class
        video_objects=[]
        for r in results.xyxy[0]:
            video_objects.append({
                "confidence": r[4].item(),
                "label": self.classes[int(r[5].item())],
                "startx": r[0].item(),
                "starty": r[1].item(),
                "endx": r[2].item(),
                "endy": r[3].item()
            })
        return video_objects

    def read_message(self,message):
        if message["timestamp"]<self.clear_frames_before:
            return
        start_time=time.time()
        video_objects=self.update(message["camera_frame"],message["timestamp"])
        self.time_sum+=time.time()-start_time
        self.sample_sum+=1
        if self.sample_sum>self.n_sample:
            gprint("Average time per tagging {} ms".format(1000*self.time_sum/self.sample_sum))
            self.sample_sum=0
            self.time_sum=0
        if len(video_objects)!=0:
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
        for i in range(len(video_objects)):
            startx=video_objects[i]["startx"]
            starty=video_objects[i]["starty"]
            endx=video_objects[i]["endx"]
            endy=video_objects[i]["endy"]
            confidence=video_objects[i]["confidence"]
            cv.rectangle(return_frame,(int(startx),int(starty)),(int(endx),int(endy)),(0,255,0),2)
            text = "{} {:.2f}%".format(video_objects[i]["label"],confidence * 100)
            Y = int(starty - 10 if starty - 10 > 10 else starty + 10)
            cv.putText(return_frame, text, (int(startx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
        self.display.update_image("object_tagger",return_frame)
