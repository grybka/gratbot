import time
import cv2 as cv
import base64
import numpy as np

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
        self.rollstate="forward"
        self.last_roll_timestamp=time.time()
        self.roll_interval=3
        self.wheel_speed=40.0

        return

    def act(self):
        self.show_video()

    def show_video(self):
        if time.time()>self.last_roll_timestamp+self.roll_interval:
            if self.rollstate=="forward":
                self.last_roll_timestamp=time.time()
                self.comms.set_intention( ["wheel_motor","speed","SET" ], self.wheel_speed)
                self.rollstate="backward"
            else:
                self.last_roll_timestamp=time.time()
                self.comms.set_intention( ["wheel_motor","speed","SET" ], -self.wheel_speed )
                self.rollstate="forward"
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
                x=bytes(image,encoding='utf-8')
                x=base64.b64decode(x)
                x=np.frombuffer(x,dtype=np.uint8)
                myframe=cv.imdecode(x,cv.IMREAD_COLOR)
                self.onstate="start"
                return myframe
        return None
