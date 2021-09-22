
import time
import numpy as np
import logging
from scipy.optimize import linear_sum_assignment
from underpinnings.id_to_name import id_to_name
from underpinnings.MotionCorrection import MotionCorrection
from Gyrus import ThreadedGyrus
from scipy.optimize import linear_sum_assignment

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
#logger.setLevel(logging.INFO)

def bbox_to_xywh(bbox):
    return [0.5*(bbox[0]+bbox[1]),0.5*(bbox[2]+bbox[3]),bbox[1]-bbox[0],bbox[3]-bbox[2]]

class Tracklet:
    def __init__(self,timestamp,detection):
        self.xywh=bbox_to_xywh(detection["bbox_array"])
        self.last_timestamp=timestamp
        self.last_label=detection["label"]
        self.status="PROBATION"
        self.n_detections=1
        self.probation_detections=3
        #here we keep a list of [timestamp, [x,y,w,h]] measurements
        self.recent_measurements_max_length=4
        self.recent_measurements=[]
        self.update(timestamp,detection)

    def update(self,timestamp,detection):
        xywh=bbox_to_xywh(detection["bbox_array"])
        #if I wanted this long, I would use the bisect alrgorithm
        #but it needs only be a few frames long
        if self.last_label!=detection["label"]:
            logger.debug("switching label from {} to {}".format(self.last_label,detection["label"]))
            self.last_label=detection["label"]
        self.recent_measurements.append( (timestamp,np.array(xywh) ) )
        self.recent_measurements.sort(key=lambda y: y[0])
        if len(self.recent_measurements)>self.recent_measurements_max_length:
            self.recent_measurements.pop(0)
        self.last_timestamp=self.recent_measurements[-1][0]
        self.n_detections+=1
        #take it off probation if enough detection
        if self.status!="PROBATION" or self.n_detections>self.probation_detections:
            self.status="DETECTED"

    def project_to_time(self,timestamp):
        #TODO try linear fit
        points=[ x[1] for x in self.recent_measurements]
        self.xywh=np.mean(points,axis=0)

    def get_xywh(self):
        return self.xywh

class TrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,include_subimages=False):
        super().__init__(broker)
        self.motion_corrector=MotionCorrection()

        self.max_assignment_cost=3.0
        self.tracklet_persist_time=1.0 #how many seconds can a a tracklet persist without being seen
        self.tracklet_lost_time=0.8 #mark a track as lost if its gone longer than this
        self.tracklet_probation_detections=3 #how many detections before a tracklet is not on probation
        self.tracklets=[]

    def get_keys(self):
        return ["detections","rotation_vector"]

    def get_name(self):
        return "TrackerGyrus"

    def read_message(self,message):
        self.motion_corrector.read_message(message)
        if "detections" in message:
            offset_x,offset_y=self.motion_corrector.get_offset_and_update(message["image_timestamp"])
            self.update_tracklets(message["image_timestamp"],message["detections"])

    def update_tracklets(self,timestamp,detections):
        for track in self.tracklets:
            track.project_to_time(timestamp)
        cost_matrix=np.zeros( [len(detections),len(self.tracklets)])
        for i in range(len(detections)):
            for j in range(len(self.tracklets)):
                cost_matrix[i,j]=self.get_score(timestamp,detections[i],self.tracklets[j])
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        leftover_is=np.setdiff1d(np.arange(len(detections)),row_ind)
        leftover_js=np.setdiff1d(np.arange(len(self.tracklets)),col_ind)
        for i in range(len(row_ind)):
            if cost_matrix[row_ind[i],col_ind[i]]>self.max_assignment_cost:
                np.append(leftover_is,row_ind[i])
                np.append(leftover_js,col_ind[i])
            else:
                self.tracklets[col_ind[i]].update(timestamp,detections[row_ind[i]])
        leftover_detections=[ detections[i] for i in leftover_is]
        leftover_tracks=[ self.tracklets[j] for j in leftover_js]
        for detection in leftover_detections:
            #propose new track
            self.propose_new_track(timestamp,detection)
        self.handle_missed_tracks(timestamp,leftover_tracks)

    def get_score(self,timestamp,detection,track):
        x,y,w,h=bbox_to_xywh(detection["bbox_array"])
        tx,ty,tw,th=track.get_xywh()
        score=0
        #position
        score+=(x-tx)**2/tw**2+(y-ty)**2/th**2
        #scale
        scale_unc=0.1
        score+=(w-tw)**2/(scale_unc*tw)**2+(h-th)**2/(scale_unc*th)**2
        #label
        label_correct_score=0.4 #80 percent likely
        label_incorrect_score=3.2 #20 percent likely
        if track.last_label==detection["label"]:
            score+=label_correct_score
        else:
            score+=label_incorrect_score
        return score/3.0

    def propose_new_track(self,timestamp,detection):
        logger.debug("adding new track")
        self.tracklets.append(Tracklet(timestamp,detection))

    def handle_missed_tracks(self,timestamp,tracks):
        dead_tracks=[]
        for track in tracks:
            if timestamp-track.last_timestamp>self.tracklet_lost_time:
                track.status="LOST"
            if timestamp-track.last_timestamp>self.tracklet_persist_time:
                dead_tracks.append(track)
        for track in dead_tracks:
            self.tracklets.remove(track)
