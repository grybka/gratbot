import time
import cv2 as cv
import base64
import numpy as np
from GratbotVideoConnectionUV4L import GratbotVideoConnectionUV4L
import logging


logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)

class GratbotBehavior:
    def __init__(self, comms):
        self.comms = comms
    def act(self):
        return
    def shut_down(self):
        return

class DisplayCamera(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.onstate="start"
        self.record_frame_period=5
        self.last_image_timestamp=time.time()

        return

    def act(self):
        return self.get_image()

    def get_image(self):
        #returns an image if a new one is available
        #otherwise none if waiting for one
        if self.onstate=="start":
            #send a request for a new image
            self.image_request_timestamp=time.time()
            self.comms.set_intention( ["camera","image","GET" ], 1 )
            self.onstate="waiting"
            return None
        elif self.onstate=="waiting":
            image,image_timestamp=self.comms.get_state("camera/image/GET")
            #print("polling: {}".format(image_timestamp))
            if image_timestamp is not None and image_timestamp>self.image_request_timestamp:
                #print("received frame after {} seconds".format(image_timestamp-self.image_request_timestamp))
                self.last_image_timestamp=image_timestamp
                x=bytes(image,encoding='utf-8')
                x=base64.b64decode(x)
                x=np.frombuffer(x,dtype=np.uint8)
                myframe=cv.imdecode(x,cv.IMREAD_COLOR)
                #if last_image_timestamp-time.time()>self.record_frame_period:
                #    cv.imwrite(time.strftime("images/%Y%m%d-%H%M%S.jpg"),myframe)
                #    record_frame_period=time.time()
                self.onstate="start"
                return myframe
        return None

    def get_image_timestamp(self):
        return self.last_image_timestamp

class DisplayCameraUV4L(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.onstate="start"
        self.record_frame_period=5
        self.last_image_timestamp=time.time()

        logging.info("Connecting to Gratbot video")
        self.video = GratbotVideoConnectionUV4L("http://10.0.0.4:8080/stream/video.mjpeg")
        frame_width = self.video.cap.get(cv.CAP_PROP_FRAME_WIDTH)
        frame_height = self.video.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        logging.info("video {} x {}".format(frame_width,frame_height))
        self.fps_frames=10
        self.fps_timer=time.time()
        self.fps_counter=0
        self.fps=0
        return

    def act(self):
        return self.get_image()

    def get_image(self):
        #returns an image if a new one is available
        #otherwise none if waiting for one
        myframe,mytime=self.video.read()
        self.last_image_timestamp=mytime
        if myframe is not None:
            self.fps_counter+=1
            if self.fps_counter>=self.fps_frames:
                self.fps=self.fps_frames/(time.time()-self.fps_timer)
                self.fps_timer=time.time()
                self.fps_counter=0
        return myframe

    def tag_with_fps(self,frame):
        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(frame, "FPS: {}".format(self.fps), (7, 70), font, 3, (100, 255, 0), 3, cv.LINE_AA)
        return frame

    def get_image_timestamp(self):
        return self.last_image_timestamp

    def shut_down(self):
        logging.info("stopping video")
        self.video.stop()
        logging.info("stopped video")
