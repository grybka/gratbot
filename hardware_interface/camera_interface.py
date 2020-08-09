"""Camera Controls"""
from hardware import GratbotSpimescape
from hardware import create_hardware_item
import threading
import time
import numpy as np
import logging
import base64
from hardware import _all_gratbot_spimescapes
from picamera import PiCamera
import cv2 


class GratbotCamera(GratbotSpimescape):


    def __init__(self, datastruct, hardware):
      self.camera=PiCamera()
      self.resolution=(640,480)
      self.camera.resolution=(self.resolution[0],self.resolution[1])
      self.camera.framerate=30
      self.camera.start_preview()

    def acquire_image(self):
      image=np.empty((self.resolution[0]*self.resolution[1]*3),dtype=np.uint8)
      #self.camera.capture(image,'bgr')
      self.camera.capture(image,'bgr',use_video_port=True)
      image=image.reshape((self.resolution[1],self.resolution[0],3))
      return image

    def set(self, endpoint, value):
      return None

    def get(self, endpoint):
        if endpoint=="image":
            logging.debug("exposure speed {}".format(self.camera.exposure_speed))
            logging.debug("framerate {}".format(self.camera.framerate))
            start_time=time.time()
            image=self.acquire_image()
            stop_time=time.time()
            logging.debug("acquiring image took {} seconds".format(stop_time-start_time))
            #_,encoded=cv2.imencode('.png',image)
            _,encoded=cv2.imencode('.jpg',image)
            encoded=encoded.tobytes()
            x=base64.b64encode(encoded)
            stop_time=time.time()
            logging.debug("acquiring and compressing image took {} seconds".format(stop_time-start_time))
            return x.decode('utf-8')
        raise Exception("unknown camera endpoint {}".format(endpoint))

_all_gratbot_spimescapes["Camera"]=GratbotCamera
