
import time
import numpy as np
import logging

import uuid
from scipy.optimize import linear_sum_assignment
from underpinnings.id_to_name import id_to_name
from underpinnings.MotionCorrection import MotionCorrection

from Gyrus import ThreadedGyrus

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)
#logger.setLevel(logging.DEBUG)

def bbox_to_xywh(bbox):
    return [0.5*(bbox[0]+bbox[1]),0.5*(bbox[2]+bbox[3]),bbox[1]-bbox[0],bbox[3]-bbox[2]]


class Tracklet:
    def __init__(self,timestamp,detection):
        self.xywh=bbox_to_xywh(detection["bbox_array"])
        self.last_timestamp=timestamp
        self.last_subimage=detection["subimage"]
        self.id=uuid.uuid1()
        self.last_label=detection["label"]
        #the probability that this thing moves on its own
        self.status="PROBATION"
        self.n_detections=0
        self.probation_detections=4
        #here we keep a list of [timestamp, [x,y,w,h]] measurements
        #self.recent_measurements_max_length=4
        #self.recent_measurements=[]
        self.last_measurement=None
        self.last_measurement_time=None
        self.last_spatial_array=None
        self.update(timestamp,detection)
        logger.debug("my xywh {}".format(self.xywh))

    def update(self,timestamp,detection):
        xywh=bbox_to_xywh(detection["bbox_array"])
        #if I wanted this long, I would use the bisect alrgorithm
        #but it needs only be a few frames long
        if self.last_label!=detection["label"]:
            logger.debug("switching label from {} to {}".format(self.last_label,detection["label"]))
            self.last_label=detection["label"]
        self.last_subimage=detection["subimage"]
        self.last_measurement=np.array(xywh)
        self.xywh=self.last_measurement
        self.last_measurement_time=timestamp
        #self.recent_measurements.append( [timestamp,np.array(xywh) ] )
        #self.recent_measurements.sort(key=lambda y: y[0])
        if "spatial_array" in detection:
            self.last_spatial_array=detection["spatial_array"]
        self.n_detections+=1
        #take it off probation if enough detection
        if self.status!="PROBATION" or self.n_detections>self.probation_detections:
            #if self.status=="PROBATION":
            #    logger.debug("{} is off probation with {} detections".format(id_to_name(self.id),self.n_detections))
            self.status="DETECTED"

    def project_to_time(self,timestamp):
        if self.status=="EXITED":
            return
        #do nothing

    def update_self_motion(self,xoffset,yoffset):
        ...

    def get_xywh(self):
        return self.xywh

    def account_for_offset(self,x_offset,y_offset):
        self.last_measurement[0]+=x_offset
        self.last_measurement[1]+=y_offset

class FastBadTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,include_subimages=False,detection_name="detections",confidence_trigger=0.5):
        self.detection_name=detection_name
        super().__init__(broker)
        self.motion_corrector=MotionCorrection()

        self.min_confidence_for_new_track=confidence_trigger
        self.max_assignment_cost=2.0
        self.tracklet_persist_time=1.0 #how many seconds can a a tracklet persist without being seen
        self.tracklet_persist_time_on_probation=0.5
        self.tracklet_lost_time=0.75 #mark a track as lost if its gone longer than this
        self.tracklet_probation_detections=3 #how many detections before a tracklet is not on probation
        self.tracklets=[]

        #reporting
        self.report_time=2.0
        self.last_report=0

    def get_keys(self):
        return [self.detection_name,"rotation_vector"]

    def get_name(self):
        return "TrackerGyrus"+self.detection_name

    def read_message(self,message):
        self.motion_corrector.read_message(message)
        if "detections" in message and len(message["detections"])!=0:
            offset_x,offset_y=self.motion_corrector.get_offset_and_update(message["image_timestamp"])
            self.update_tracklets(message["image_timestamp"],message["detections"],offset_x,offset_y)
            if time.time()-self.last_report>self.report_time:
                self.last_report=time.time()
                logger.debug("Track Report")
                for tracklet in self.tracklets:
                    logger.debug("{} {}".format(tracklet.last_label,id_to_name(tracklet.id)))


    def update_tracklets(self,timestamp,detections,offset_x,offset_y):
        #if detections[0]["label"]!="face":
        #    logger.debug("detections: {}".format([ d["label"] for d in detections]))

        leftover_detection_indices=[ i for i in range(len(detections)) ]
        leftover_tracks=[]
        for track in self.tracklets:
            #TODO include offset here
            track.account_for_offset(offset_x,offset_y)
            track.project_to_time(timestamp)
            best_cost=np.inf
            assignment=-1
            for j in leftover_detection_indices:
                cost=self.get_score(timestamp,detections[j],track)
                if cost<best_cost and cost<self.max_assignment_cost:
                    assignment=j
                    best_cost=cost
            if assignment!=-1: #I can assign it
                leftover_detection_indices.remove(assignment)
                track.update(timestamp,detections[assignment])
            else:
                leftover_tracks.append(track)




        #cost_matrix=np.zeros( [len(detections),len(self.tracklets)])
        #for i in range(len(detections)):
        #    for j in range(len(self.tracklets)):
        #        cost_matrix[i,j]=self.get_score(timestamp,detections[i],self.tracklets[j])

        #row_ind, col_ind = linear_sum_assignment(cost_matrix)
        #leftover_is=np.setdiff1d(np.arange(len(detections)),row_ind)
        #leftover_js=np.setdiff1d(np.arange(len(self.tracklets)),col_ind)
        #for i in range(len(row_ind)):
        #    if cost_matrix[row_ind[i],col_ind[i]]>self.max_assignment_cost:
        #        #logger.debug("assignment cost too high {}".format(cost_matrix[row_ind[i],col_ind[i]]))
        #        #logger.debug("trying to hit {},{} - {}".format(self.tracklets[row_ind[i]].xywh[0],self.tracklets[row_ind[i]].xywh[1],id_to_name(self.tracklets[row_ind[i]].id)))
        #        leftover_is=np.append(leftover_is,row_ind[i])
        #        leftover_js=np.append(leftover_js,col_ind[i])
        #    else:
        #        #logger.debug("assigned with score {}".format(cost_matrix[row_ind[i],col_ind[i]]))
        #        self.tracklets[col_ind[i]].update(timestamp,detections[row_ind[i]])


        leftover_detections=[ detections[i] for i in leftover_detection_indices]
        for detection in leftover_detections:
            #propose new track
            self.propose_new_track(timestamp,detection)
        self.handle_missed_tracks(timestamp,leftover_tracks)

        track_message=[]
        for track in self.tracklets:
            det_item={}
            x,y,w,h=track.get_xywh()
            det_item["bbox_array"]=[ x-w/2, x+w/2,y-h/2,y+h/2]
            det_item["center"]=[x,y]
            det_item["id"]=track.id
            det_item["info"]=track.status
            det_item["label"]=track.last_label
            det_item["subimage"]=track.last_subimage
            det_item["seen_frames"]=track.n_detections
            if track.last_spatial_array is not None:
                det_item["spatial_array"]=[track.last_spatial_array[0]/1000,track.last_spatial_array[1]/1000,track.last_spatial_array[2]/1000]
            track_message.append(det_item)
        message_out={"tracks": track_message,
                     "timestamp": time.time(),
                     "image_timestamp": timestamp,
                     "offset": [offset_x,offset_y],
                     "detection_name": self.detection_name}
        self.broker.publish(message_out,["tracks"])

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
        if detection["confidence"]<self.min_confidence_for_new_track:
            logger.debug("Ignoring a {} because confidence is {}".format(detection["label"],detection["confidence"]))
            return
        new_track=Tracklet(timestamp,detection)
        logger.debug("adding new track {} that is a {}".format(id_to_name(new_track.id),new_track.last_label))
        logger.debug("at {},{} with confidence {} at {}".format(new_track.xywh[0],new_track.xywh[1],detection["confidence"],timestamp))
        self.tracklets.append(new_track)

    def handle_missed_tracks(self,timestamp,tracks):
        dead_tracks=[]
        for track in tracks:
            if track.xywh[0]<0 or track.xywh[0]>1 or track.xywh[1]<0 or track.xywh[1]>1:
                if track.status!="EXITED":
                    logger.debug("{} has exited view".format(id_to_name(track.id)))
                track.status="EXITED"
            if timestamp-track.last_timestamp>self.tracklet_lost_time:
                track.status="LOST"
            if timestamp-track.last_timestamp>self.tracklet_persist_time or (track.status=="PROBATION" and timestamp-track.last_timestamp>self.tracklet_persist_time_on_probation):
                dead_tracks.append(track)
        for track in dead_tracks:
            logger.debug("removing track {} at {}".format(id_to_name(track.id),timestamp))
            self.tracklets.remove(track)
