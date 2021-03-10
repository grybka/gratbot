#ImageTracks a thing in 2 dimensions assuming imperfect vision

import numpy as np
import cv2 as cv
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class VisualTracker:

    def __init__(self,haarcascade_filename):
        #TODO upgrade these to initialization arguments
        self.existance_past_frames=10
        self.existance_frame_count=4 # I have to see target in 4 out of the past 10 frames for it to count as real
        self.target_seen_array=np.zeros(self.existance_past_frames)
        #--Initialize the kalman filters---
        self.kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        self.kfy = KalmanFilter(dim_x=2,dim_z=1) #ypos, yvel,      ymeas
        self.kfx.x=np.array([0, 0]) # initially at 0,0 with 0 velocity
        self.kfy.x=np.array([0, 0])
        #transition matrix (x=x0+v*t)
        self.kfx.F=np.array([ [1.0,1.0],
                [0.0,1.0] ])
        self.kfy.F=np.array([ [1.0,1.0],
                [0.0,1.0] ])
        #measurement function (only position)
        self.kfx.H=np.array([[1.0,0.0]])
        self.kfy.H=np.array([[1.0,0.0]])
        #covariance matrix
        self.kfx.P *= 1000. #I guess I figure this is big 
        self.kfy.P *= 1000. #I guess I figure this is big 
        #Fundamental maesurement noise
        self.kfx.R =np.array([[10.]])
        self.kfy.R =np.array([[10.]])
        self.kfx.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=10)
        self.kfy.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=10)
        #---load the opencv haar cascade----
        self.target_cascade = cv.CascadeClassifier(haarcascade_filename)

    def handle_next_frame(self,frame):
        #only handles a single target.  TODO handle multiple targets
        targets = self.target_cascade.detectMultiScale(frame, 1.3, 5)
        self.kfx.predict()
        self.kfy.predict()
        self.target_seen_array=np.delete(self.target_seen_array,0)
        if len(targets)>0:
            print("SEEN!")
            self.target_seen_array=np.append(self.target_seen_array,1.0)
            (x,y,w,h)=targets[0]
            x_center=x+w/2
            y_center=y+w/2
            self.kfx.update([x_center])
            self.kfy.update([y_center])
        else:
            self.target_seen_array=np.append(self.target_seen_array,0.0)

    def test_existance(self):
        return np.sum(self.target_seen_array)>=self.existance_frame_count

    def highlight_frame(self,frame):
        if self.test_existance():
            cv.rectangle(frame,(int(self.kfx.x[0]-10),int(self.kfy.x[0])-10),(int(self.kfx.x[0])+10,int(self.kfy.x[0])+10),(255,0,0),2)






