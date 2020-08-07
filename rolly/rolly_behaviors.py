import time
import cv2 as cv
import base64

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
        return

    def act(self):
        if self.onstate="start":
            #send a request for a new image
            self.image_request_timestamp=time.time()
            self.comms.set_intention( ["camera","image","get" ], 1 )
            self.onstate="waiting"
            return None
        elif self.onstate=="waiting":
            image,image_timestamp=self.comms.get_state("camerage/image")
            if image_timestamp>self.image_request_timestamp:
                myframe=cv2.imdecode(base64.b64decode(image))
                self.onstate="start"
                return myframe
        return None
