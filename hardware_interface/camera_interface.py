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
      self.last_image=None
      self.last_image_encoded=None
      self.fps_timer=0
      self.fps_reporting=10
      self.wait_time=0.01
      #taking pictures is actually time consuming, so we're not going to use the normal shared spimescape thread
      self.loop_counter=0
      self.thread = threading.Thread(target=self._daemon_loop)
      self.image_lock=threading.Lock()
      self.thread.daemon = True
      self.thread_should_quit = False
      self.thread.start()

    def acquire_image(self):
      image=np.empty((self.resolution[0]*self.resolution[1]*3),dtype=np.uint8)
      #self.camera.capture(image,'bgr')
      self.camera.capture(image,'bgr',use_video_port=True)
      image=image.reshape((self.resolution[1],self.resolution[0],3))
      return image

    def _daemon_loop(self):
        self.loop_counter+=1
        #logging.debug("exposure speed {}".format(self.camera.exposure_speed))
        #logging.debug("framerate {}".format(self.camera.framerate))
        start_time=time.time()
        self.fps_timer+=start_time-self.last_image_timestamp
        if self.loop_counter%self.fps_reporting==0:
            self.fps=1/self.fps_timer
            logging.debug("fps: ".format(self.fps))
            self.fps_timer=0
        self.last_image=self.acquire_image()
        stop_time=time.time()
        logging.debug("acquiring image took {} seconds".format(stop_time-start_time))
        _,encoded=cv2.imencode('.jpg',image)
        encoded=encoded.tobytes()
        x=base64.b64encode(encoded)
        #stop_time=time.time()
        self.image_lock.acquire()
        self.last_image_encoded=x.decode('utf-8')
        self.last_image_timestamp=start_time
        self.image_lock.release()
        logging.debug("acquiring and compressing image took {} seconds".format(time.time()-start_time))
        time.sleep(self.wait_time)

    def set(self, endpoint, value):
      return None

    def get(self, endpoint):
        if endpoint=="image":
            self.image_lock.acquire()
            x=self.last_image_encoded
            self.image_lock.release()
            return x
        raise Exception("unknown camera endpoint {}".format(endpoint))

_all_gratbot_spimescapes["Camera"]=GratbotCamera
