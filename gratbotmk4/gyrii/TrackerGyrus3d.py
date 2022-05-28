
import time
import numpy as np
import logging
import uuid
from scipy.optimize import linear_sum_assignment
from underpinnings.LocalPositionLog import LocalPositionLog
from collections import deque
from Gyrus import ThreadedGyrus
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
from pyquaternion import Quaternion
#logger.setLevel(logging.WARNING)

class Tracklet:
    def __init__(self):
        self.id=uuid.uuid1()
        #position measurements xyz_map
        self.map_positions=deque([],maxlen=4)
        #position relative to camera
        self.camera_positions=deque([],maxlen=4)
        #bounding boxes
        self.frame_positions=deque([],maxlen=4)
        #labels
        self.labels=deque([],maxlen=4)
        #times
        self.timestamps=deque([],maxlen=4)
        #how stable is the track
        self.stability=-4
        #color
        self.color_features=deque([],maxlen=4)

    def mark_missed(self):
        self.stability-=1

    def update(self,timestamp,detection,map_xyz,det_uv):
        self.map_positions.appendleft(map_xyz)
        self.camera_positions.appendleft(detection["spatial_array"])
        self.frame_positions.appendleft(detection["bbox_array"])
        self.labels.appendleft(detection["label"])
        self.timestamps.appendleft(timestamp)
        self.stability=min(10,self.stability+detection["confidence"])
        self.color_features.appendleft(det_uv)


class TrackerGyrus3d(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.position_log=LocalPositionLog()
        self.tracklets=[]

        self.logp_label_misid=4.0
        #track match score cutoff
        self.track_match_score_cutoff=1.5
        #likelyhood of missing a detection
        #at what stability do I drop tracks
        self.drop_track_stability=-30
        self.last_image=None

    def get_keys(self):
        return ["detections","rotation_vector","image"]

    def get_name(self):
        return "TrackerGyrus3d"

    def read_message(self,message):
        self.position_log.read_message(message) #handles rotation vector and motors
        start_time=time.time()
        if "image" in message:
            self.last_image=message["image"]
        if self.last_image is None:
            return
        if "detections" in message:
            #discard if I've gotten too behind
            too_behind_seconds=0.05
#            if start_time-message["timestamp"]>too_behind_seconds:
#                logger.debug("discarding detections because tracker is too far behind")
#                return
            self.update_tracklets(message["detections"],message["timestamp"])
            #report
            self.publish_report(message["image_timestamp"])

    def update_tracklets(self,detections,timestamp):
        #logger.debug("{} traclktes active".format(len(self.tracklets)))
        #logger.debug("{} detections a".format(len(detections)))
        #remove dead tracks
        to_drop=[]
        for i in range(len(self.tracklets)):
            if self.tracklets[i].stability<self.drop_track_stability:
                to_drop.append(self.tracklets[i])
        for i in range(len(to_drop)):
            self.tracklets.remove(to_drop[i])
        #calculate the U,V of each image
        det_uv_list=[]
        for i in range(len(detections)):
            det_uv_list.append(self.get_color_feature(self.last_image,detections[i]["bbox_array"]))
        #calcuate the map positions of all my detections
        quat=Quaternion(self.position_log.pointing)
        rot=quat.rotation_matrix
        det_xyz_list=[]
        for i in range(len(detections)):
            yxz=detections[i]["spatial_array"]
            xyz=np.array([yxz[1],yxz[0],yxz[2]])
            new_xyz=rot@xyz
            det_xyz_list.append(new_xyz)
        #calculate cost matrix
        cost_matrix=np.zeros( [len(detections),len(self.tracklets)])
        for i in range(len(detections)):
            for j in range(len(self.tracklets)):
                cost_matrix[i,j]=self.get_score(detections[i],self.tracklets[j],det_xyz_list[i],det_uv_list[i])
        #assign tracks
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        leftover_is=np.setdiff1d(np.arange(len(detections)),row_ind)
        leftover_js=np.setdiff1d(np.arange(len(self.tracklets)),col_ind)
        for i in range(len(row_ind)):
            if cost_matrix[row_ind[i],col_ind[i]]>self.track_match_score_cutoff:
                #inadequate match
                #TODO this is insufficient.  I should not have two objects at the same location with the same label
                #logger.debug("inadequate match {} to {}".format(self.tracklets[col_ind[i]].labels[0],detections[row_ind[i]]["label"]))
                #logger.debug("inadequate match with score {}".format(cost_matrix[row_ind[i],col_ind[i]]))
                leftover_is=np.append(leftover_is,row_ind[i])
                leftover_js=np.append(leftover_js,col_ind[i])
            else:
                #logger.debug("assigned with score {}".format(cost_matrix[row_ind[i],col_ind[i]]))
                self.tracklets[col_ind[i]].update(timestamp,detections[row_ind[i]],det_xyz_list[row_ind[i]],det_uv_list[row_ind[i]])
        leftover_tracks=[ self.tracklets[j] for j in leftover_js]
        for i in leftover_is: #detections
            self.propose_new_track(timestamp,detections[i],det_xyz_list[i],det_uv_list[i])
        for i in leftover_js: #tracks
            self.tracklets[i].mark_missed()


    def publish_report(self,image_timestamp):
        track_message=[]
        for track in self.tracklets:
            det_item={}
            det_item["bbox_array"]=track.frame_positions[0]
            #logger.debug("last detbox {}".format(track.frame_positions[0]))
            det_item["camera_position"]=track.camera_positions[0]
            det_item["map_position"]=track.map_positions[0]
            det_item["id"]=track.id
            det_item["label"]=track.labels[0]
            det_item["stability"]=track.stability
            track_message.append(det_item)
        message_out={"tracks": track_message,
                     "timestamp": time.time(),
                     "image_timestamp": image_timestamp,
                     "detection_name": "detections"}
        self.broker.publish(message_out,["tracks"])

    def get_score(self,detection,tracklet,det_xyz,det_uv):
        score=0
        bbox_t=tracklet.frame_positions[0]
        bbox_d=detection["bbox_array"]
        tracklet_size=(bbox_t[1]-bbox_t[0])*(bbox_t[3]-bbox_t[2])
        det_size=(bbox_d[1]-bbox_d[0])*(bbox_d[3]-bbox_d[2])
        minx=max(bbox_d[0],bbox_t[0])
        maxx=min(bbox_d[1],bbox_t[1])
        miny=max(bbox_d[2],bbox_t[2])
        maxy=min(bbox_d[3],bbox_t[3])
        overlap=(maxx-minx)*(maxy-miny)/tracklet_size
        ##
        color_diff=((tracklet.color_features[0]-det_uv)**2).sum()
        #
        distance2=((det_xyz-tracklet.map_positions[0])**2).sum()/4e5
        if overlap>0:
            score-=overlap
        if detection["label"]==tracklet.labels[0]:
            score-=1
        if color_diff<2:
            score-=(color_diff+2)
        if distance2<1:
            score-=(1-distance2)
        #logger.debug("matching detection {} to track {}".format(detection["label"],tracklet.labels[0]))
        #logger.debug("distance2 {}".format(distance2))
        #logger.debug("overlap: {}".format(overlap))
        #logger.debug("color diff: {}".format(color_diff-2))
        #logger.debug("score {}".format(score+3))
        #logger.debug("color difference {}".format(color_diff))
        return score+3
        #score=0
        ##map objects closer to one another
        #distance2=((det_xyz-tracklet.map_positions[0])**2).sum()
        #dist_chisq=distance2/2e5 #determined experimentally
        #if dist_chisq<1:
        #    score+=2-dist_chisq
        ##logger.debug("xyzdistance {}".format(dist_chisq))
        ##TODO should I care about timestamp here?  probably
        ##bounding box size
        #tframe=tracklet.frame_positions[0]
        #dframe=detection["bbox_array"]
        #dsize=(dframe[1]-dframe[0])*detection["spatial_array"][2]
        #tsize=(tframe[1]-tframe[0])*tracklet.camera_positions[0][2]
        #sizechisq=(dsize-tsize)**2/8e4 #determined experimentally
        #chisq+=sizechisq
        ##logger.debug("size {}".format(sizechisq))
        #if detection["label"]!=tracklet.labels[0]:
        #    chisq+=self.logp_label_misid
        ##TODO maybe some color features here.  Something fast
        #return chisq/3 #three things so far
        return score/3

    def propose_new_track(self,timestamp,detection,map_xyz,det_uv):
        new_tracklet=Tracklet()
        new_tracklet.update(timestamp,detection,map_xyz,det_uv)
        self.tracklets.append(new_tracklet)

    def get_color_feature(self,image,bbox):
        xmin=int(max(bbox[0],0)*image.shape[1])
        xmax=int(min(bbox[1],1)*image.shape[1])
        ymin=int(max(bbox[2],0)*image.shape[0])
        ymax=int(min(bbox[3],1)*image.shape[0])
        my_bbox=[xmin,xmax,ymin,ymax]
        color_avg=np.mean(np.mean(image[my_bbox[0]:my_bbox[1],my_bbox[2]:my_bbox[3]],axis=0),axis=0)
        B,G,R=color_avg
        U = -0.148 * R - 0.291 * G + 0.439 * B + 128;
        V =  0.439 * R - 0.368 * G - 0.071 * B + 128;
        return np.array([U,V])

    #extract color feature from bbox
    #  Y =  0.257 * R + 0.504 * G + 0.098 * B +  16; #brightness do not use
    #  U = -0.148 * R - 0.291 * G + 0.439 * B + 128;
    #  V =  0.439 * R - 0.368 * G - 0.071 * B + 128;
