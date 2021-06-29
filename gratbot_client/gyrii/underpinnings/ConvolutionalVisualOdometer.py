#Convolutional Visual Odometer

import cv2 as cv
import time
import numpy as np
from scipy.linalg import pinvh

class ConvolutionalVisualOdometer:
    def __init__(self,margin=30):
        self.margin=margin #how many pixels around the center to track

        self.camera_hfov=48.8*(2*np.pi)/360 #from spec sheet
        self.last_frame=None
        self.warmup_start_time=0
        self.x_velocity=0
        self.y_velocity=0
        self.velocity_timebase=0
        self.last_frame_timestamp=0

    def get_pose_change(self,xoffset):
        f_theta=640/self.camera_hfov
        L=1.0
        f_x=f_theta/L
        #xoffset=f * dtheta + (F/L) * dx
        #L is actually unknown.  Let's take a charactericistic length of 1m, ideally this runs into uncentainty
        #chisquare=  ( xoffset - f*dtheta - (f/L) *dx ) **2  / sigmaxfosset^2+ (dtheta^2)/(s_theta^2)+(dx/s_x)^2
        #= f^2*dtheta^2/s_o^2+(f/L)^2*dx^2 /s_o^2+ (f^2/L)dtheta*dx/s_o +dtheta^2/s_theta^2+dx^2/s_x^2
        #So equations tosolve are
        s_o=3 #pixel uncertainty
        #s_o=0.1 #pixel uncertainty
        s_x= 0.5/30.0 #likely position change, in meters
        #s_x= 0.001/30.0 #likely position change, in meters
        s_theta=np.radians(12) #likely angle change, in radians
        A= [ [ f_theta**2+(s_o/s_theta)**2     ,   + f_theta*f_x             ],
             [ +f_theta*f_x                        ,  f_x**2+(s_o/s_x)**2] ]
        B= [ [ xoffset*f_theta],
             [ xoffset*f_x]]
        x_center=np.dot(pinvh(A),B) #column vector

        covariance=np.array([ [ 1/((f_theta/s_o)**2+(1/s_theta)**2), -1/(f_theta*f_x) ],
                              [ -1/(f_theta*f_x)                   , 1/((f_x/s_o)**2+(1/s_x)**2) ] ])
        #covariance=np.array([ [ (s_o/f_theta)**2+(s_theta)**2, -1/(f_theta*f_x) ],
        #                      [ -1/(f_theta*f_x)                   , (s_o/f_x)**2+(s_x)**2 ] ])
        #covariance=np.array([ [ (f_theta/s_o)**2+(1/s_theta)**2, -f_theta*f_x ],
        #                      [ -f_theta*f_x                   , (f_x/s_o)**2+(1/s_x)**2 ] ])
        return np.array([x_center[0][0],x_center[1][0]]),covariance

    def update(self,frame,timestamp):
        #reduce frame to BW and downsample
        dim=(160,100)
        margin=self.margin
        resized_frame=cv.resize(frame,dim)
        #toshow=resized_frame.copy()
        #cv.drawMarker(toshow,(self.x_center,self.y_center),(255,0,0))
        #toshow=cv.resize(toshow,(640,480))
        #self.display.update_image("visual_odometer_cross",toshow)
        bw_frame=cv.cvtColor(resized_frame,cv.COLOR_BGR2GRAY)
        #self.display.update_image("visual_odometer_bw",bw_frame)
        #convolve with previous image
        offset_x=0
        offset_y=0
        if self.last_frame is not None:
            #print("bw shape {}".format(bw_frame.shape))
            centerbit=bw_frame[ int(dim[1]/2-margin):int(dim[1]/2+margin),int(dim[0]/2-margin):int(dim[0]/2+margin) ]
            #self.display.update_image("visual_odometer_center",centerbit)
            output=cv.matchTemplate(self.last_frame,centerbit,cv.TM_SQDIFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv.minMaxLoc(output)

            pwrong=0.1
            npoints=margin*margin
            sigma=pwrong/np.sqrt(npoints)
            probsize=5
            #print("max val, min val {} , {}".format(max_val,min_val))
            #print("min loc {}".format(min_loc))
            #print("max loc {}".format(max_loc))
            #print("outptu shape {} ".format(output.shape))
            allsum=0
            xsum=0
            ysum=0
            for i in range(-5,5):
                for j in range(-5,5):
                    pos1=min_loc[1]+j
                    pos2=min_loc[0]+i
                    if pos1>=0 and pos1<output.shape[0] and pos2>=0 and pos2<output.shape[1]:
                        prob=np.exp(-output[pos1,pos2]/sigma)
                        xsum+=prob*pos2
                        ysum+=prob*pos1
                        allsum+=prob
            #print("weigthed x, xsum all sum {} {} {}".format(xsum,ysum,allsum))
            weighted_x=xsum/allsum
            weighted_y=ysum/allsum

            #print("{} {}".format(i,output[min_loc[1],min_loc[0]+i]))
            #offset_x=int((min_loc[0]-(self.last_frame.shape[1]/2-margin))*frame.shape[0]/160)
            #offset_y=int((min_loc[1]-(self.last_frame.shape[0]/2-margin))*frame.shape[1]/100)
            offset_x=(weighted_x-(self.last_frame.shape[1]/2-margin))*frame.shape[0]/160
            offset_y=(weighted_y-(self.last_frame.shape[0]/2-margin))*frame.shape[1]/100

            #self.time_sum+=time.time()-start_time
            #self.sample_sum+=1
            #if self.sample_sum>=self.n_sample:
                #gprint("maxval at {}".format(ind))
                #gprint("moment along 0 is {}".format(moment(output,axis=None)))
            #    gprint("average convolution time {} ms".format(1000*self.time_sum/self.sample_sum))
            #    gprint("number of dropped frames {}".format(self.n_dropped_frames))
            #    self.sample_sum=0
            #    self.time_sum=0
        else:
            self.warmup_start_time=time.time()
        self.last_frame=bw_frame
        if time.time()>self.warmup_start_time+2.0:
            self.velocity_timebase=timestamp-self.last_frame_timestamp
            self.x_velocity=offset_x/self.velocity_timebase
            self.y_velocity=offset_y/self.velocity_timebase
            self.last_frame_timestamp=timestamp
            return int(offset_x),int(offset_y) #it has a habit of the first few frames being wild.  give it some startup time
        else:
            return 0,0

class ConvolutionalVisualOdometerx3:

    def __init__(self,margin=30):
        self.margin=margin #how many pixels around the center to track

        self.camera_hfov=48.8*(2*np.pi)/360 #from spec sheet
        self.last_frame=None


    def update(self,frame,timestamp):
        #reduce frame to BW and downsample
        dim=(160,100)
        margin=self.margin
        resized_frame=cv.resize(frame,dim)
        bw_frame=cv.cvtColor(resized_frame,cv.COLOR_BGR2GRAY)
        #so I'd like to space my frames like this |margin[frame1][frame2][frame3]margin|
        if self.last_frame is not None:
            centerbit=bw_frame[ int(dim[1]/2-margin):int(dim[1]/2+margin),int(dim[0]/2-margin):int(dim[0]/2+margin) ]
            leftbit=bw_frame[ int(dim[1]/2-margin):int(dim[1]/2+margin),int(dim[0]/2-margin):int(dim[0]/2+margin) ]
