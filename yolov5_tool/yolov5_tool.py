import torch.backends.cudnn as cudnn
import sys
import cv2 as cv
#sys.path.append('C:/Users/grybk/projects/gratbot/hexapod')
sys.path.append('C:/Users/grybk/projects/gratbot/yolov5_tool')
sys.path.append('C:/Users/grybk/projects/gratbot/yolov5_tool/yolov5')

import numpy as np
from yolov5.models.experimental import *
from yolov5.utils.datasets import *
from yolov5.utils.torch_utils import *
from yolov5.utils.general import *
import logging

class yolov5_tool:
    def __init__(self):
        #self.classes=["purple_ball","box"]
        self.classes=[]
        self.initialized=False
        return

    def initialize(self,weight_path):
        self.device=''
        self.imgsz=640
        # Initialize
        #self.device = torch_utils.select_device(self.device)
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'
        # Load Model
        self.model=torch.load(weight_path,map_location=self.device)['model'].float().fuse().eval()
        self.imgsz = check_img_size(self.imgsz, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()

        img = torch.zeros((1,3,self.imgsz,self.imgsz),device=self.device)
        _ = self.model(img.half() if self.half else img) if self.device.type!= 'cpu' else None

        #get clas names
        self.classes=self.model.module.names if hasattr(self.model,'module') else self.model.names
        print("classis is {}".format(self.classes))
        self.initialized=True

    def detect(self,img):
        #prep image
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img=img.half() if self.half else img.float()
        img /= 255.0 #
        if img.ndimension() ==3:
            img=img.unsqueeze(0)

        #run inference
        augment=True
        pred=self.model(img,augment=True)[0]
        # Apply NMS
        conf_thres=0.4
        #conf_thres=0.1
        iou_thres=0.5
        #iou_thres=0.1
        agnostic=True
        classes=[]

        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic)

        video_objects=[]
        for i, det in enumerate(pred):
            if det is not None and len(det):
                for *xyxy,conf,cls in det:
                    # Rescale boxes from img_size to im0 size
                    #det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                    video_objects.append({
                        "confidence": conf.item(),
                        "label": self.classes[int(cls.item())],
                        "startx": xyxy[0].item(),
                        "starty": xyxy[1].item(),
                        "endx": xyxy[2].item(),
                        "endy": xyxy[3].item()
                    })
        return video_objects

    def draw_object_bboxes(self,video_frame,video_objects):
        #logging.info("drawing bboxes")
        for i in range(len(video_objects)):
            startx=video_objects[i]["startx"]
            starty=video_objects[i]["starty"]
            endx=video_objects[i]["endx"]
            endy=video_objects[i]["endy"]
            confidence=video_objects[i]["confidence"]
            cv.rectangle(video_frame,(int(startx),int(starty)),(int(endx),int(endy)),(0,255,0),2)
            text = "{} {:.2f}%".format(video_objects[i]["label"],confidence * 100)
            Y = int(starty - 10 if starty - 10 > 10 else starty + 10)
            cv.putText(video_frame, text, (int(startx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
