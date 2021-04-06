from Gyrus import ThreadedGyrus
import cv2 as cv
import numpy as np
import threading
import logging
import time

class GratbotUV4LToBrokerThread:

    def __init__(self,broker,stream_address,display_loop,simulation_mode=False):
        self.broker=broker
        self.stream_address=stream_address
        self.simulation_mode=simulation_mode
        self.display_loop=display_loop
        self.end_called=False
        if not self.simulation_mode:
            self.cap = cv.VideoCapture(stream_address)
        else: #SIMULATION MODE
            #self.sim_frame=cv.imread("images/image_20210324-144207.png")
            self.sim_frame=cv.imread("images/image_20210324-144157.png")
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self):
        while not self.end_called:
            if not self.simulation_mode:
                ret,grabbed_frame=self.cap.read()
                if ret is True:
                    self.broker.publish({"timestamp": time.time(),"camera_frame": grabbed_frame},"camera_frame")
                    self.display_loop.update_image("camera_frame",grabbed_frame)
                else:
                    logging.error("Some problem with camera frame")
            else:
                time.sleep(1) #TODO in simulation mode put some fixed picture
                self.broker.publish({"timestamp": time.time(),"camera_frame": self.sim_frame},"camera_frame")
                self.display_loop.update_image("camera_frame",self.sim_frame)
        logging.info("Video Thread Stopped")

    def stop(self):
        self.end_called=True
        self.thread.join()
        self.thread=None
        logging.info("Releasing Video")
        if not self.simulation_mode:
            self.cap.release()

    def __exit__(self, exc_type, exc_value, traceback):
        if self.thread is not None:
            self.stop()
