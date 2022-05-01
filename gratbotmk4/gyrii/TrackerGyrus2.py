
import time
import numpy as np
import logging
import uuid
from scipy.optimize import linear_sum_assignment
from underpinnings.id_to_name import id_to_name
from underpinnings.MotionCorrection import MotionCorrection
from Gyrus import ThreadedGyrus
from scipy.optimize import linear_sum_assignment

logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
logger.setLevel(logging.INFO)

#Behavior -
#A detection is either identified as part of an existing tracklet, or as a new track, depending on match score
#A new tracklet is 'on probation' for probation_detection_frames
#If a tracklet on probation does not reach probation_detection_frames in tracklet_persist_time_on_probation seconds, it will be discarded
#if a tracklet not on probation is not detected in a frame it is declared MISSING
#if a MISSING tracklet's projected position is off-screen, it is declared EXITED
#if a MISSING tracklet has not had a detection in tracklet_persist_time, it is declared LOST and removed after report
#tracklets that are not on probation are reported with DETECTED, MISSING, EXITED, or LOST status

def bbox_to_xywh(bbox):
    return [0.5*(bbox[0]+bbox[1]),0.5*(bbox[2]+bbox[3]),bbox[1]-bbox[0],bbox[3]-bbox[2]]

class Tracklet:
    def __init__(self,timestamp,detection):
        self.xywh=bbox_to_xywh(detection["bbox_array"])
        self.vxvy=[0,0]
        self.last_timestamp=timestamp
        #self.last_subimage=detection["subimage"]
        self.last_detection_bbox=[0,0,0,0]
        self.id=uuid.uuid1()
        self.last_label=detection["label"]
        #the probability that this thing moves on its own
        self.status="PROBATION" #One of PROBATION DETECTED MISSING EXITED or LOST
        self.n_detections=0
        self.probation_detections=4
        #here we keep a list of [timestamp, [x,y,w,h]] measurements
        self.recent_measurements_max_length=4
        self.recent_measurements=[]
        self.last_spatial_array=None
        self.update(timestamp,detection)
        logger.debug("my xywh {}".format(self.xywh))

    def update(self,timestamp,detection):
        self.last_detection_bbox=detection["bbox_array"]
        xywh=bbox_to_xywh(detection["bbox_array"])
        #if I wanted this long, I would use the bisect alrgorithm
        #but it needs only be a few frames long
        if self.last_label!=detection["label"]:
            logger.debug("switching label from {} to {}".format(self.last_label,detection["label"]))
            self.last_label=detection["label"]
        #self.last_subimage=detection["subimage"]
        self.recent_measurements.append( [timestamp,np.array(xywh) ] )
        self.recent_measurements.sort(key=lambda y: y[0])
        if len(self.recent_measurements)>self.recent_measurements_max_length:
            self.recent_measurements.pop(0)
        if "spatial_array" in detection:
            self.last_spatial_array=detection["spatial_array"]
        self.last_timestamp=self.recent_measurements[-1][0]
        self.n_detections+=1
        #take it off probation if enough detection
        if self.status!="PROBATION" or self.n_detections>self.probation_detections:
            self.status="DETECTED"

    def project_to_time(self,timestamp):
        if self.status=="EXITED":
            return
        points=np.array([ x[1] for x in self.recent_measurements])
        mean_xywh=np.mean(points,axis=0)
        #if False:
        if len(self.recent_measurements)>2:
            #TODO try linear fit
            times=np.array([ x[0] for x in self.recent_measurements])

            #suppose it was moving with constant velocity
            xfit=np.polynomial.polynomial.Polynomial.fit(times,points[:,0],1)
            yfit=np.polynomial.polynomial.Polynomial.fit(times,points[:,1],1)
            xcoeffs=xfit.convert().coef
            ycoeffs=yfit.convert().coef
            if len(xcoeffs)<2:
                xv=0
            else:
                xv=xcoeffs[1]
            if len(ycoeffs)<2:
                yv=0
            else:
                yv=ycoeffs[1]
            self.vxvy=[xv,yv]
            xpred=xfit(timestamp)
            ypred=yfit(timestamp)
            #logger.debug("mean {} {}".format(mean_xywh[0],mean_xywh[1]))
            #logger.debug("pval {} {}".format(xpred,ypred))
            #never let tracks travel more than mostly off screen
            w=mean_xywh[2]
            h=mean_xywh[3]
            #xpred=np.clip(xpred,-w/2,1+w/2)
            #ypred=np.clip(ypred,-h/2,1+h/2)
            self.xywh=np.array([xpred,ypred,w,h,])
        else:
            self.xywh=mean_xywh

    def update_self_motion(self,xoffset,yoffset):
        ...

    def get_xywh(self):
        return self.xywh

    def account_for_offset(self,x_offset,y_offset):
        #logger.debug("offest {} {}".format(x_offset,y_offset))
        for measurement in self.recent_measurements:
            measurement[1][0]+=x_offset
            measurement[1][1]+=y_offset

class TrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,include_subimages=False,detection_name="detections",confidence_trigger=0.5):
        self.detection_name=detection_name
        super().__init__(broker)
        self.motion_corrector=MotionCorrection()

        self.min_confidence_for_new_track=confidence_trigger
        self.max_assignment_cost=2.5
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
        start_time=time.time()
        if "detections" in message and len(message["detections"])!=0:
            #offset_x,offset_y=self.motion_corrector.get_offset_and_update(message["image_timestamp"])
            offset_x,offset_y=self.motion_corrector.get_offset_and_update(message["image_timestamp"])
            #offset_x,offset_y=0,0
            self.update_tracklets(message["image_timestamp"],message["detections"],offset_x,offset_y)
            if time.time()-self.last_report>self.report_time:
                time_lag=message["image_timestamp"]-self.motion_corrector.get_latest_timestamp()
                self.last_report=time.time()
                logger.info("Track Report")
                logger.info("number of detections this frame {}".format(len(message["detections"])))
                #logger.info("Time Lag : {}".format(time_lag))
                logger.info("Time Lag : {}".format(time.time()-start_time))
                for tracklet in self.tracklets:
                    logger.info("{} {}: {}  vx,vy ({},{})".format(tracklet.last_label,id_to_name(tracklet.id),tracklet.status,tracklet.vxvy[0],tracklet.vxvy[1]))


    def update_tracklets(self,timestamp,detections,offset_x,offset_y):
        #if detections[0]["label"]!="face":
        #    logger.debug("detections: {}".format([ d["label"] for d in detections]))


        #offset x and y should be in radians
        #track positions are in vido full scale
        #camera is 69 x 55 degrees, allegedly (or is it 73?  it is different in different places)
        for track in self.tracklets:
            #TODO include offset here
            track.account_for_offset(offset_x,offset_y)
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
                #logger.debug("assignment cost too high {}".format(cost_matrix[row_ind[i],col_ind[i]]))
                #logger.debug("trying to hit {},{} - {}".format(self.tracklets[row_ind[i]].xywh[0],self.tracklets[row_ind[i]].xywh[1],id_to_name(self.tracklets[row_ind[i]].id)))
                leftover_is=np.append(leftover_is,row_ind[i])
                leftover_js=np.append(leftover_js,col_ind[i])
            else:
                #logger.debug("assigned with score {}".format(cost_matrix[row_ind[i],col_ind[i]]))
                self.tracklets[col_ind[i]].update(timestamp,detections[row_ind[i]])

        leftover_detections=[ detections[i] for i in leftover_is]
        leftover_tracks=[ self.tracklets[j] for j in leftover_js]
        for detection in leftover_detections:
            #propose new track
            self.propose_new_track(timestamp,detection)
        self.mark_missed_tracks(timestamp,leftover_tracks)

        track_message=[]
        for track in self.tracklets:
            if track.status=="PROBATION":
                continue #Don't report probationary tracks
            det_item={}
            x,y,w,h=track.get_xywh()
            det_item["bbox_array"]=[ x-w/2, x+w/2,y-h/2,y+h/2]
            det_item["last_detection_bbox"]=track.last_detection_bbox
            det_item["center"]=[x,y]
            det_item["recent_measurements"]=track.recent_measurements
            det_item["id"]=track.id
            det_item["info"]=track.status
            det_item["label"]=track.last_label
            #det_item["subimage"]=track.last_subimage
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
        #Confidence
        score+=self.min_confidence_for_new_track/(detection["confidence"]+0.10)

        if track.last_label==detection["label"]:
            score+=label_correct_score
        else:
            score+=label_incorrect_score
        return score/4.0

    def propose_new_track(self,timestamp,detection):
        if detection["confidence"]<self.min_confidence_for_new_track:
            logger.debug("Ignoring a {} because confidence is {}".format(detection["label"],detection["confidence"]))
            return
        new_track=Tracklet(timestamp,detection)
        logger.debug("adding new track {} that is a {}".format(id_to_name(new_track.id),new_track.last_label))
        logger.debug("at {},{} with confidence {} at {}".format(new_track.xywh[0],new_track.xywh[1],detection["confidence"],timestamp))
        self.tracklets.append(new_track)

    def mark_missed_tracks(self,timestamp,tracks):
        for track in tracks:
            if track.status!="EXITED" and track.status!="PROBATION":
                track.status="MISSING"
            if track.xywh[0]<0 or track.xywh[0]>1 or track.xywh[1]<0 or track.xywh[1]>1:
                track.status="EXITED"
            if track.status=="PROBATION":
                if timestamp-track.last_timestamp>self.tracklet_persist_time_on_probation:
                    track.status="LOST"
            elif track.status=="MISSING" or track.status=="EXITED":
                if timestamp-track.last_timestamp>self.tracklet_lost_time:
                    track.status="LOST"

    def handle_missed_tracks(self,timestamp,tracks):
        dead_tracks=[]
        for track in tracks:
            if track.status=="LOST":
                dead_tracks.append(track)
        for track in dead_tracks:
            logger.debug("removing track {} at {}".format(id_to_name(track.id),timestamp))
            self.tracklets.remove(track)
