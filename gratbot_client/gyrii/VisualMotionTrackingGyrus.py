
#Visual Odometry Mk2
from Gyrus import ThreadedGyrus
import cv2 as cv
import time
import numpy as np
from scipy.signal import convolve
from scipy.stats import moment
#from scipy.ndimage import convolve
from gyrii.underpinnings.GratbotLogger import gprint

class VisualMotionTrackingGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.clear_frames_before=0
        self.display=display
        #profiling
        self.n_dropped_frames=0
        self.n_sample=10
        self.time_sum=0
        self.sample_sum=0
        self.fgbg = cv.createBackgroundSubtractorMOG2()
        super().__init__(broker)

    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "VisualMotionTrackingGyrus"


    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                self.n_dropped_frames+=1
                return
            self.update(message["camera_frame"],message["timestamp"])
            self.clear_frames_before=time.time()

    def update(self,frame,timestamp):
        start_time=time.time()
        dim=(160,100)
        margin=30
        resized_frame=cv.resize(frame,dim)
        fgmask=self.fgbg.apply(resized_frame)
        contours,heirarchy=cv.findContours(fgmask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        contour_counter=0
        for c in contours:
            if cv.contourArea(c) <50:
                continue
            contour_counter+=1
            (x,y,w,h)=cv.boundingRect(c)
            cv.rectangle(resized_frame, (x,y),(x+w,y+h),(0,55,0),2)

        #TODO publish a list of detections here!

        #gprint("{} contours found ".format(contour_counter))
        #toshow=cv.resize(fgmask,(640,480))
        toshow=cv.resize(resized_frame,(640,480))
        self.display.update_image("motion_tracking",toshow)
        #bw_frame=cv.cvtColor(resized_frame,cv.COLOR_BGR2GRAY)

        self.time_sum+=time.time()-start_time
        self.sample_sum+=1
        if self.sample_sum>=self.n_sample:
            #gprint("maxval at {}".format(ind))
            #gprint("moment along 0 is {}".format(moment(output,axis=None)))
            gprint("average fg time {} ms".format(1000*self.time_sum/self.sample_sum))
            gprint("n dropped frames per frame {}".format(self.n_dropped_frames/self.sample_sum))
            self.n_dropped_frames=0
            self.sample_sum=0
            self.time_sum=0
