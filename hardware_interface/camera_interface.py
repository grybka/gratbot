
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


class GratbotCamera(GratbotSpimescape):


    def __init__(self, datastruct, hardware):
      self.camera=PiCamera()
      self.resolution=(640,480)
      camera.resolution=(self.resolution[0],self.resolution[1])
      camera.framerate=24
      camera.start_preview()

    def acquire_image(self):
      image=np.empty((self.resolution[0]*self.resolution[1]*3),dtype=np.uint8)
      camera.capture(image,'bgr')
      image=image.reshape((self.resolution[1],self.resolution[0],3))
      return image

    def set(self, endpoint, value):
      return None

    def get(self, endpoint):
      return None
      
_all_gratbot_spimescapes["Camera"]=GratbotCamera
