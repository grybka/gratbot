import torch.backends.cudnn as cudnn
import sys
sys.path.append('C:/Users/grybk/projects/gratbot/hexapod')
sys.path.append('C:/Users/grybk/projects/gratbot/hexapod/yolov5_tool')
sys.path.append('C:/Users/grybk/projects/gratbot/hexapod/yolov5_tool/yolov5')

import numpy as np
from yolov5_tool.yolov5.models.experimental import *
from yolov5_tool.yolov5.utils.datasets import *
from yolov5_tool.yolov5.utils.utils import *

class yolov5_tool:
    def __init__(self):
        self.classes=["purple_ball","box"]
        return

    def initialize(self,weight_path):
        self.device=''
        self.imgsz=640
        # Initialize
        self.device = torch_utils.select_device(self.device)
        self.half = self.device.type != 'cpu'
        # Load Model
        self.model=torch.load(weight_path,map_location=self.device)['model'].float().fuse().eval()
        self.imgsz = check_img_size(self.imgsz, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()

        img = torch.zeros((1,3,self.imgsz,self.imgsz),device=self.device)
        _ = self.model(img.half() if self.half else img) if self.device.type!= 'cpu' else None

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
        #conf_thres=0.4
        conf_thres=0.1
        #iou_thres=0.5
        iou_thres=0.1
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
