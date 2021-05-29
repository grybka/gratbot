
#Visual Odometry Mk2
from Gyrus import ThreadedGyrus
import cv2 as cv
import time
import numpy as np
#from scipy.signal import convolve
#from scipy.stats import moment
#from scipy.ndimage import convolve
from scipy.linalg import pinvh
from gyrii.underpinnings.GratbotLogger import gprint

class VisualOdometerConvGyrus(ThreadedGyrus):
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
        self.x_center=80
        self.y_center=50

        self.camera_hfov=48.8*(2*np.pi)/360 #from spec sheet
        self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet


    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "VisualOdometerConv"

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
        dim=(160,100)
        margin=30
        resized_frame=cv.resize(frame,dim)
        toshow=resized_frame.copy()
        cv.drawMarker(toshow,(self.x_center,self.y_center),(255,0,0))
        toshow=cv.resize(toshow,(640,480))
        self.display.update_image("visual_odometer_cross",toshow)
        bw_frame=cv.cvtColor(resized_frame,cv.COLOR_BGR2GRAY)
        #self.display.update_image("visual_odometer_bw",bw_frame)
        #convolve with previous image
        if self.last_frame is not None:
            centerbit=bw_frame[ int(dim[1]/2-margin):int(dim[1]/2+margin),int(dim[0]/2-margin):int(dim[0]/2+margin) ]
            self.display.update_image("visual_odometer_center",centerbit)
            output=cv.matchTemplate(self.last_frame,centerbit,cv.TM_SQDIFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(output)
            offset_x=int(min_loc[0]-(self.last_frame.shape[1]/2-margin))
            offset_y=int(min_loc[1]-(self.last_frame.shape[0]/2-margin))

            self.time_sum+=time.time()-start_time
            self.sample_sum+=1
            if self.sample_sum>=self.n_sample:
                #gprint("maxval at {}".format(ind))
                #gprint("moment along 0 is {}".format(moment(output,axis=None)))
                gprint("average convolution time {} ms".format(1000*self.time_sum/self.sample_sum))
                gprint("number of dropped frames {}".format(self.n_dropped_frames))
                self.sample_sum=0
                self.time_sum=0
        self.last_frame=bw_frame
