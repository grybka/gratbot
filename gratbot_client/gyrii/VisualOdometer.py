#Visual Odometry
from Gyrus import ThreadedGyrus
import cv2 as cv
import time
import numpy as np
from gyrii.underpinnings.GratbotLogger import gprint

class VisualOdometerGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        n_keypoints=100
        self.clear_frames_before=0
        self.orb=cv.ORB_create(nfeatures=n_keypoints)
        self.display=display
        self.last_features_kp=None
        self.last_features_des=None
        #self.orb=cv.ORB()

        ##FLANN MATCHER
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm = FLANN_INDEX_LSH,
                    table_number = 6,
                    key_size = 12,
                    multi_probe_level = 1)
        search_params = dict(checks = 50)
        self.flann = cv.FlannBasedMatcher(index_params, search_params)
        ####
        super().__init__(broker)

        #profiling
        self.n_sample=10
        self.time_sum=0
        self.sample_sum=0

    def get_keys(self):
        return [ "camera_frame" ]

    def get_name(self):
        return "VisualOdometer"

    def my_recover_pose(self,kp1,kp2):
        sumdiff=0
        for i in range(len(kp1)):
            sumdiff+=kp2[i][0]-kp1[i][0]
        gprint("sumdiff is {}".format(sumdiff/len(kp1)))

    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                return
            start_time=time.time()
            self.update(message["camera_frame"],message["timestamp"])
            self.time_sum+=time.time()-start_time
            self.sample_sum+=1
            if self.sample_sum>=self.n_sample:
                #gprint("maxval at {}".format(ind))
                #gprint("moment along 0 is {}".format(moment(output,axis=None)))
                gprint("average convolution time {} ms".format(1000*self.time_sum/self.sample_sum))
                self.sample_sum=0
                self.time_sum=0
            self.clear_frames_before=time.time()

#    def get_closest(self,kp1,kp2):
#        kp1_i=0
#        kp2_i=0
#        best_dist=1e9
#        best_x1=0
#        best_x2=0
#        while True:
#            if kp1_i>=len(kp1) or kp2_i>=len(kp2):
#                break
#            x1=kp1[kp1_i]
#            x2=kp2[kp2_i]
#            if abs(x2-x1)<best_dist:
#                best_dist=x2-x1
#                best_x1=x1
#                best_x2=x2
#            if x1<x2:
#                kp1_i+=1
#            else
#                kp2_i+=1
#        return best_dist,best_x1,best_x2


    #def my_own_matcher(self,kp1,kp2):
    #    height=480
    #    n_bins=48
    #    bins=np.linspace(0,480,n_bins)
    #    #step 1, sort keypoints into horizontal layer
    #    kp1_ys=np.array( [ x.pt[1] for x in kp1 ])
    #    kp1_ys=np.array( [ x.pt[1] for x in kp2 ])
    #    kp1_rows=np.digitize(kp1_ys,bins)
    #    kp2_rows=np.digitize(kp2_ys,bins)
    #    #step 2, find the two closest points in each layer
    #    for onlayer in len(bins):
    #        kp1_here=sorted(kp1[kp1_rows[i]])
    #        kp2_here=sorted(kp2[kp2_rows[i]])
    #        dist,x1,x2=self.get_closest(kp1_here,kp2_here)


    def update(self,frame,timestamp):
        framecopy=frame.copy()
        #gprint("framecopy size {}".format(framecopy.shape))

        kp,des=self.orb.detectAndCompute(frame,None)
        #gprint("kp len {}".format(len(kp)))
        if len(kp)<=5: #TODO make controllable
            return
        #gprint("example keypoints {}".format(kp[0].pt))
        if self.last_features_des is not None:
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

            pts1=np.array(pts1)
            pts2=np.array(pts2)
            self.my_recover_pose(pts1,pts2)
            #gprint("pts2 length {}".format(len(pts2)))
            #self.camera_focal_length_pixels=531
            #E,mask=cv.findEssentialMat(pts2,pts1,focal=self.camera_focal_length_pixels,pp=(640,480))
            #_, R, t, Mask= cv.recoverPose(E,pts2,pts1,focal=self.camera_focal_length_pixels,pp=(640,480))
            #retval,_,_,Qx,Qy,Qz=cv.RQDecomp3x3(R)
            #gprint("R is {}".format(R))
            #gprint(" ti is {}".format(t))
            #gprint("ANgles (degrees) {}".format(retval))
            #np.degrees(Qx),np.degrees(Qy),np.degrees(Qz)))
            #TODO what do I do with T and R?

        cv.drawKeypoints(framecopy,kp,framecopy,color=(0,255,0),flags=0)
        self.display.update_image("visual_odometer",framecopy)
        self.last_features_des=des
        self.last_features_kp=kp
