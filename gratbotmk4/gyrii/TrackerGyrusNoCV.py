#no opencv tracker

# will track only one of each type
import cv2
import uuid
import logging
from collections import deque
from underpinnings.id_to_name import id_to_name
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
from filterpy.common import kinematic_kf,Q_discrete_white_noise

from Gyrus import ThreadedGyrus

logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
logger.setLevel(logging.INFO)

#TODO if something moved plausibly offscreen, then it should persist longer


def get_closest_value(timestamp,mylist):
        #for mylist ordered by [time,value], return the value that is closest to the input timestamp
        if len(mylist)==0:
            return None
        if timestamp<=mylist[0][0]:
            return mylist[0][1]

        first_val=mylist[0]
        second_val=None
        for val in mylist:
            if timestamp>val[0]:
                second_val=val
            else:
                first_val=val
        if abs(first_val[0]-timestamp)<abs(second_val[0]-timestamp):
            return first_val[1]
        else:
            return second_val[1]


class MotionCorrection: #to correct image frames from heading changes
    def __init__(self,max_recent_history=20):
        self.max_recent_history=max_recent_history
        self.gyros=deque([],maxlen=self.max_recent_history)
        self.headings=deque([],maxlen=self.max_recent_history) #from gyro integration
        self.last_used_heading=0
        self.angle_heading_slope=-1.515
        self.z_gyro_index=0

    def read_message(self,message):
        if 'packets' in message: #rotation, etc
            if len(self.headings)==0: #for the first packet received
                self.headings.append([message['packets'][0]["gyroscope_timestamp"],0 ])
            for packet in message["packets"]:
                self.gyros.append( [packet["gyroscope_timestamp"],packet["gyroscope"]])
                    #TODO I could do a fancier integration
                next_heading=self.headings[-1][1]+packet["gyroscope"][self.z_gyro_index]*(packet["gyroscope_timestamp"]-self.headings[-1][0])
                self.headings.append( [packet["gyroscope_timestamp"],next_heading])

    def get_offset_and_update(self,image_timestamp):
        if len(self.headings)==0:
            return 0
        closest_heading_vec=get_closest_value(image_timestamp,self.headings)
        delta_heading=closest_heading_vec-self.last_used_heading
        self.last_used_heading=closest_heading_vec #this is the update part
        offset=delta_heading*self.angle_heading_slope
        return offset

class TrackerGyrusTrackedObject:
    def __init__(self):
        self.id=uuid.uuid1()


        #info from detections
        self.last_label=None
        self.last_depth=None
        self.last_det_bbox=None


        #tracking misses
        self.last_success=True
        self.frames_without_detection=0
        #kalman filtering
        #x and y in full span of the camera
        #z is in meters and only updated with depth
        dt=1/30.0
        self.kfx=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfy=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfz=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfx.P=1e9*np.eye(2) #covariance
        self.kfy.P=1e9*np.eye(2) #covariance
        self.kfz.P=1e9*np.eye(2) #covariance

    def update_kf_from_time(self,dt):
        #assume 1 pixel creep per second
        self.kfx.Q=Q_discrete_white_noise(2,dt=dt,var=1)
        self.kfy.Q=Q_discrete_white_noise(2,dt=dt,var=1)
        #assume 1 cm creep per second
        self.kfy.Q=Q_discrete_white_noise(2,dt=dt,var=0.01**2)
        self.kfx.predict()
        self.kfy.predict()
        self.kfz.predict()

    def update_with_detection(self,det,image):
        box=self.get_det_bbox(det)
        self.last_depth=det["spatial_array"][2]/1000 #in m
        self.last_det_bbox=box
        self.last_label=det['label']
        x_update,y_update=box[0]/image.shape[1],box[1]/image.shape[0]
        #print("x update, y update {},{} for {}".format(x_update,y_update,det["label"]))
        #assume 2 pixel resolution for a detection
        self.kfx.R=np.array( [[ (2/image.shape[1])**2 ]])
        self.kfx.update(x_update)
        self.kfy.R=np.array( [[ (2/image.shape[0])**2 ]])
        self.kfy.update(y_update)
        #assume 5 cm resolution for depth
        self.kfz.R=np.array( [[ 0.05**2 ]])
        self.kfz.update(self.last_depth)
        self.frames_without_detection=0

    def init_with_detection(self,image,det):
        self.shape=image.shape

        self.update_with_detection(det,image)

    def get_centroid(self):
        return (int(self.kfx.x[0][0]*self.shape[1]),int(self.kfy.x[0][0]*self.shape[1]))

    def get_det_bbox(self,det):
        x=int(0.5*(det["bbox_array"][0]+det["bbox_array"][1])*self.shape[1])
        y=int(0.5*(det["bbox_array"][2]+det["bbox_array"][3])*self.shape[0])
        w=int((det["bbox_array"][1]-det["bbox_array"][0])*self.shape[1])
        h=int((det["bbox_array"][3]-det["bbox_array"][2])*self.shape[1])
        return (x,y,w,h)

    def get_bestguess_bbox(self):
        centroid=self.get_centroid()
        return [centroid[0],centroid[1],self.last_det_bbox[2],self.last_det_bbox[3]]

    def get_center(self):
        return [ self.kfx.x[0][0],self.kfy.x[0][0],self.kfz.x[0][0] ]

    def get_velocity(self):
        return [ self.kfx.x[1][0],self.kfy.x[1][0],self.kfz.x[1][0] ]



class TrackerGyrusNoCV(ThreadedGyrus):
    def __init__(self,broker):
        self.motion_corrector=MotionCorrection()

        self.trackers=[]
        self.last_image_timestamp=0
        #conditions to remove tracker
        self.max_frames_without_detection=30 #1 second
        self.max_frames_offscreen=90 #3 seconds
        super().__init__(broker)
        #for timing
        self.report_spf_count=30
        self.spf_count=0
        self.spf_sum_time=0


    def get_keys(self):
        return ["image","rotation_vector"]

    def get_name(self):
        return "TrackerGyrus"

    def getTracks(self):
        ret=[]
        for tracker in self.trackers:
            mess={}
            bb=tracker.get_bestguess_bbox()
            mess["bbox_array"]=[ int(bb[0]-bb[2]/2),int(bb[0]+bb[2]/2),int(bb[1]-bb[3]/2),int(bb[1]+bb[3]/2)]
            mess["center"]=tracker.get_center()
            mess["velocity"]=tracker.get_velocity()
            mess["missed_frames"]=tracker.frames_without_detection
            mess["label"]=tracker.last_label
            mess["id"]=tracker.id
            ret.append(mess)
        return ret

    def update_trackers(self,image,image_timestamp,offset):
        if self.last_image_timestamp==0:
            self.last_image_timestamp=image_timestamp
            return
        dt=image_timestamp-self.last_image_timestamp
        dead_trackers=[]

        for tracker in self.trackers:
            tracker.update_kf_from_time(dt)
            tracker.kfx.x[0]+=offset
            if 0<tracker.kfx.x[0]<1.0:
                if tracker.frames_without_detection>self.max_frames_without_detection:
                    dead_trackers.append(tracker)
            else:
                if tracker.frames_without_detection>self.max_frames_offscreen:
                    dead_trackers.append(tracker)
            tracker.frames_without_detection+=1

        for tracker in dead_trackers:
            print("dropping {} with missed frames {}".format(tracker.last_label,tracker.frames_without_detection))
            self.trackers.remove(tracker)



        self.last_image_timestamp=image_timestamp

    def get_match_score(self,tracker,det,imageshape):
        #so what's important?  Distance match, size match, and label match
        #metric: if you are halfway across the screen and the wrong label, let's call that a mismatch, even if same size
        #that's a distance of like 200 pixels.
        wrong_label_cost=0.3
        dist_cost_weight=0.7/200
        size_cost_weight=0.2/200
        label_cost=0
        if tracker.last_label!=det["label"]:
            label_cost=wrong_label_cost
        det_bbox=tracker.get_det_bbox(det)
        tracker_bbox=tracker.get_bestguess_bbox()
        dist_cost=dist_cost_weight*np.sqrt( (det_bbox[0]-tracker_bbox[0])**2+(det_bbox[1]-tracker_bbox[1])**2)
        size_cost=size_cost_weight*np.sqrt( (det_bbox[2]-tracker_bbox[2])**2+(det_bbox[3]-tracker_bbox[3])**2)
        return label_cost+dist_cost+size_cost

    def match_trackers_to_detections(self,dets,image):
          #figure out best batches between the two
        cost_matrix=np.zeros( [len(dets),len(self.trackers)])
        for i in range(len(dets)):
            for j in range(len(self.trackers)):
                det=dets[i]
                tracker=self.trackers[j]
                cost_matrix[i,j]=self.get_match_score(tracker,det,image.shape)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        leftover_dets=np.setdiff1d(np.arange(len(dets)),row_ind)
        leftover_trackers=np.setdiff1d(np.arange(len(self.trackers)),col_ind)
        #logger.debug("Out of {} detections and {} tracks".format(len(dets),len(self.trackers)))
        #logger.debug("I've match up {} detections and tracks ".format(len(row_ind)))
        #logger.debug("leaving {} new detections and {} unsupported tracks".format(len(leftover_dets),len(leftover_trackers)))
        #deal with matches
        self.max_assignment_cost=1.0
        for i in range(len(row_ind)):
            if cost_matrix[row_ind[i],col_ind[i]]>self.max_assignment_cost:
                np.append(leftover_dets,row_ind[i])
                np.append(leftover_trackers,col_ind[i])
            else:
                self.trackers[col_ind[i]].update_with_detection(dets[row_ind[i]],image)
        #deal with leftover detections
        for i in range(len(leftover_dets)):
            new_track=TrackerGyrusTrackedObject()
            new_track.init_with_detection(image,dets[leftover_dets[i]])
            self.trackers.append(new_track)
        #Do I need to deal with leftover tracks?  Maybe not, maybe deweight them TODO
        for i in range(len(leftover_trackers)):
            tracker=self.trackers[leftover_trackers[i]]
            if tracker.last_success:
                logger.debug("I have a tracker success but no detection for {}".format(id_to_name(tracker.id)))
            else:
                logger.debug("I have neither a tracker nor a detection for {}".format(id_to_name(tracker.id)))


    def read_message(self,message):
        self.motion_corrector.read_message(message)

        if "image" in message and "tracks" not in message:
            #logger.debug("Image Recieved")
            start_time=time.time()

            #handle heading correction
            offset=self.motion_corrector.get_offset_and_update(message["image_timestamp"])

            self.update_trackers(message["image"],message["image_timestamp"],offset)

            #don't bother with detections if in a saccade, they're probably not right even if there
            if "detections" in message:
                #logger.debug("handling detections")
                self.match_trackers_to_detections(message["detections"],message["image"])

            message_out={"tracks": self.getTracks(),"timestamp": time.time(),"image_timestamp": message["image_timestamp"],"offset": offset}
            self.broker.publish(message_out,["tracks"])

            #timing
            self.spf_count+=1
            self.spf_sum_time+=time.time()-start_time
            if self.spf_count>=self.report_spf_count:
                logger.info("Tracking time fer frame {} ms".format(1000*self.spf_sum_time/self.spf_count))
                logger.info("Number of Tracked obects now {}".format(len(self.trackers)))
                self.spf_sum_time=0
                self.spf_count=0
                for tracker in self.trackers:
                    ...
                    #print("Track: {} is a {} ".format(id_to_name(tracker.id),tracker.last_label))
                    #print("Location: {}".format(tracker.get_center()))
                    #print("Velocity: {}".format(tracker.get_velocity()))
