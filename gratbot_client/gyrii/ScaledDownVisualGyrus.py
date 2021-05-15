#Visual Odometry Mk2
from Gyrus import ThreadedGyrus
import cv2 as cv
import time
import numpy as np
#from scipy.signal import convolve
#from scipy.stats import moment
#from scipy.ndimage import convolve
from gyrii.underpinnings.GratbotLogger import gprint

class ScaledDownVisualGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.clear_frames_before=0
        self.display=display
        super().__init__(broker)
        self.last_frame=None
        #profiling
        self.n_dropped_frames=0
        self.n_sample=10
        self.time_sum=0
        self.sample_sum=0
        self.target_directory="lowres_images/"
        self.last_saved_image_time=0
        self.image_counter=0

    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "VisualScaledDown"

    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                self.n_dropped_frames+=1
                return
            self.update(message["camera_frame"],message["timestamp"])
            self.clear_frames_before=time.time()

    def update(self,frame,timestamp):
        #reduce frame to BW and downsample
        start_time=time.time()
        dim=(320,240)
        #dim=(90,50)
        margin=30
        resized_frame=cv.resize(frame,dim)
        toshow=cv.resize(resized_frame,(640,480))
        #toshow=resized_frame.copy()
        self.display.update_image("visual_downscaled",toshow)
        if time.time()>self.last_saved_image_time+5:
            name="lowres_image_{}.png".format(self.image_counter)
            cv.imwrite(self.target_directory+name,resized_frame)
            self.broker.publish({"timestamp": time.time(),"saved_lowres_image": name },"notification")
            self.image_counter+=1
            self.last_saved_image_time=time.time()
        #cv.drawMarker(toshow,(self.x_center,self.y_center),(255,0,0))
        #toshow=cv.resize(toshow,(640,480))
        #self.display.update_image("visual_odometer_cross",toshow)
        #bw_frame=cv.cvtColor(resized_frame,cv.COLOR_BGR2GRAY)
        #self.display.update_image("visual_odometer_bw",bw_frame)
        #convolve with previous image
