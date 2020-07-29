import torch.backends.cudnn as cudnn

from yolov5.models.experimental import *
from yolov5.utils.datasets import *
from yolov5.utils.utils import *

class yolov5_tool:
    def __init__():
        return

    def initialize(self,weight_path):
        device=''
        weights='' # TODO this matters
        imgsz=640
        # Initialize
        device = torch_utils.select_device(device)
        half = device.type != 'cpu'
        # Load Model
        model = attempt_load(weight_path, map_location=device)  # load FP32 model
        imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size

    def detect(self,img):
        #prep image
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(device)
        img=img.half() if half else img.float()
        img /= 255.0 #
        if img.ndimension() ==3:
            img=img.unsqueeze(0)

        #run inference
        pred=model(img)[0]

        video_objects=[]
        for i, det in enumerate(pred):
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                #det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                video_objects.append({
                    "confidence": det[4],
                    "label": det[5],
                    "startx": det[0],
                    "starty": det[1],
                    "endx": det[2],
                    "endy": det[3]
                })
        return video_objects
