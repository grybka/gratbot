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
        self.record_frame_period=5
        last_image_timestamp=time.time()

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
