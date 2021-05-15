
import cv2 as cv
import time
import numpy as np
from gyrii.underpinnings.GratbotLogger import gprint

class VodometerException(Exception):
    def __init__(self,message):
        self.message=message

class Vodometer():
    def __init__(self):
        #self.n_keypoints=100
        self.n_keypoints=200
        self.orb=cv.ORB_create(nfeatures=self.n_keypoints)
        self.last_features_kp=None
        self.last_features_des=None

        ##FLANN MATCHER
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm = FLANN_INDEX_LSH,
                    table_number = 6,
                    key_size = 12,
                    multi_probe_level = 1)
        search_params = dict(checks = 50)
        self.flann = cv.FlannBasedMatcher(index_params, search_params)

    #returns number of pixels this frame is offset from last frame
    def get_offset_since_last(self,frame):
        my_frame=frame[0:240,:,:]
        kp,des=self.orb.detectAndCompute(my_frame,None)
        if len(kp)<=10: #TODO make controllable
            raise VodometerException("not enough keypoints")
        #gprint("kp {}".format(len(kp)))
        if self.last_features_des is None:
            self.last_features_des=des
            self.last_features_kp=kp
            raise VodometerException("No previous frame")
        matches = self.flann.knnMatch(self.last_features_des, des, k=2)
        # Need to draw only good matches, so create a mask
        pts1,pts2=[],[]
        #gprint("number of matches {}".format(len(matches)))
        # ratio test as per Lowe's paper
        for i,pair in enumerate(matches):
            try:
                m,n = pair
                if m.distance < 0.7*n.distance:
                    pts2.append(kp[m.trainIdx].pt)
                    pts1.append(self.last_features_kp[m.queryIdx].pt)
            except ValueError:
                pass


        self.last_features_des=des
        self.last_features_kp=kp
        if len(pts1)<5 or len(pts2)<5:
            raise VodometerException("No Match")


        pts1=np.array(pts1)
        pts2=np.array(pts2)
        deltas=pts2-pts1

        return np.mean(deltas[:,0]),np.mean(deltas[:,1]),np.std(deltas[:,0]),np.std(deltas[:,1])
