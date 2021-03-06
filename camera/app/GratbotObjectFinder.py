import threading
import cv2
import time
import logging

    #classNames = {0: 'background', 1: 'Green Light', 2: 'Lego Person', 3: 'Red Light',4: 'Speed Limit 25', 5: 'Speed Limit 40', 6: 'Stop Sign'}
class GratbotObjectFinder:
    classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}

    def __init__(self,videostream):
        self.videostream=videostream
        self.should_quit=False
        #https://github.com/rdeepc/ExploreOpencvDnn?source=post_page---------------------------
        self.model = cv2.dnn.readNetFromTensorflow('models/frozen_inference_graph.pb', 'models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
        #self.model = cv2.dnn.readNetFromTensorflow('models/mine/target_model.pb', 'models/mine/label_map.pbtxt')
        self.output_frame=[]
        self.output_frame_lock=threading.Lock()
        self.fps=0
        self.fps_lock=threading.Lock()
        self.detection_array=[] #array of (classname,x,y,w,h,confidence)
        self.detection_array_timestamp=time.time()
        self.detection_array_lock=threading.Lock()
        self._my_thread=threading.Thread(target=self._loop,daemon=True)
        self._my_thread.start()

    def id_class_name(self,class_id, classes):
        for key,value in classes.items():
            if class_id == key:
                return value

    def _loop(self):
        counter=0
        fps_count=10
        start_time=time.time()
        while not self.should_quit:
            counter+=1
            frame,frame_timestamp=self.videostream.read()
            image_height, image_width, _ = frame.shape
            #logging.info("frame shape {}".format(frame.shape))
            self.model.setInput(cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True))
            #logging.info("blobbed")
            output=self.model.forward()
            new_detection_array=[]
            #logging.info("forwarded")
            for detection in output[0,0,:,:]:
                confidence = detection[2]
                if confidence > .5:
                    class_id=detection[1]
                    class_name=self.id_class_name(class_id,self.classNames)
                    box_x = detection[3] * image_width
                    box_y = detection[4] * image_height
                    box_width = detection[5] * image_width
                    box_height = detection[6] * image_height
                    cv2.rectangle(frame, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), thickness=1)
                    #cv2.putText(frame,class_name ,(int(box_x), int(box_y+.05*image_height)),cv2.FONT_HERSHEY_SIMPLEX,(.005*image_width),(0, 0, 255))
                    cv2.putText(frame,class_name ,(int(box_x), int(box_y+.05*image_height)),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0, 0, 255))
                    new_detection_array.append( [class_name,detection[3],detection[4],detection[5],detection[6],detection[2] ] )
            self.output_frame_lock.acquire()
            self.output_frame=frame

            self.output_frame_lock.release()
            if counter%fps_count:
                now_time=time.time()
                self.fps_lock.acquire()
                self.fps=fps_count/(now_time-start_time)
                self.fps_lock.release()
                start_time=now_time
            self.detection_array_lock.acquire()
            self.detection_array=new_detection_array
            self.detection_array_timestamp=frame_timestamp
            self.detection_array_lock.release()

    def get_detection_array(self):
            #returns array of detected things of structure
            #[ classname, x,y,width,height,confidence ]
            #and the time the frame was captured
            self.detection_array_lock.acquire()
            ret_array=self.detection_array
            ret_timestamp=self.detection_array_timestamp
            self.detection_array_lock.release()
            return ret_array,ret_timestamp

    def get_processed_frame(self):
            self.output_frame_lock.acquire()
            retval=self.output_frame
            self.output_frame_lock.release()
            return retval

    def get_fps(self):
            self.fps_lock.acquire()
            retval=self.fps
            self.fps_lock.release()
            return retval

    def stop(self):
            self.should_quit=True
            self._my_thread.join()
