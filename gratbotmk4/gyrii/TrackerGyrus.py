#using opencv tracker

# will track only one of each type
import cv2
import uuid
import logging
from underpinnings.id_to_name import id_to_name
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
from filterpy.common import kinematic_kf

from Gyrus import ThreadedGyrus

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)

class TrackerGyrusTrackerCentroid:
    def __init__(self,id=None):
        self.missed_frames=0
        #self.cv_tracker=cv2.TrackerMOSSE_create()
        self.cv_tracker=cv2.TrackerKCF_create()
        self.last_bbox=(0,0,0,0)
        self.label=None
        self.last_success=True
        if id is None:
            self.id=uuid.uuid1()
        else:
            self.id=id

        self.centered_shape=[128,128]
        self.kf = kinematic_kf(dim=2, order=1, dt=1.0/30, order_by_dim=False)
        self.kf.P=1e6*np.eye(4)
        self.kf.R*=10
        self.centroid=[0,0]


    def bbox_to_tracker(self,bbox):
        return ( bbox[0]-self.centroid[0]+self.centered_shape[0],bbox[1]-self.centroid[1]+self.centered_shape[0],bbox[2],bbox[3])

    def tracker_to_bbox(self,bbox):
        return ( bbox[0]+self.centroid[0]-self.centered_shape[0],bbox[1]+self.centroid[1]-self.centered_shape[0],bbox[2],bbox[3])

    def image_to_tracker(self,image):
        padded=cv2.copyMakeBorder(image, self.centered_shape[0], self.centered_shape[0], self.centered_shape[1], self.centered_shape[1], cv2.BORDER_CONSTANT, None,  (0,0,0))
        return padded[self.centroid[0]:self.centroid[0]+2*self.centered_shape[0],self.centroid[1]:self.centroid[1]+2*self.centered_shape[1],:]

    def init_with_detection(self,image,det):
        self.last_bbox=self.get_det_bbox(det,image.shape)
        self.kf.x[0]=self.last_bbox[0]
        self.kf.x[1]=self.last_bbox[1]
        self.centroid=(self.last_bbox[0],self.last_bbox[1])
        self.label=det["label"]
        #logger.debug("init with centroid {} and adjusted bbox of {}".format(self.centroid,self.last_bbox))
        #logger.debug("image shape is going to be {}".format(self.image_to_tracker(image).shape))
        self.cv_tracker.init(self.image_to_tracker(image),self.bbox_to_tracker(self.last_bbox))

    def update(self,frame):
        self.kf.predict()
        self.centroid=(int(self.kf.x[0]),int(self.kf.x[1]))
        (self.last_success, box) = self.cv_tracker.update(self.image_to_tracker(frame))

        if not self.last_success or box==(0,0,0,0):
            self.missed_frames+=1
            self.last_success=False  #sometimes it gives a bad bounding box but reports success.
        else:
            self.missed_frames=0
            self.kf.update([self.last_bbox[0],self.last_bbox[1]])
            self.last_bbox=self.tracker_to_bbox(box)
            self.centroid=(self.last_bbox[0],self.last_bbox[1])

    def get_det_bbox(self,det,imageshape):
        x=int(0.5*(det["bbox_array"][0]+det["bbox_array"][1])*imageshape[1])
        y=int(0.5*(det["bbox_array"][2]+det["bbox_array"][3])*imageshape[0])
        w=int((det["bbox_array"][1]-det["bbox_array"][0])*imageshape[1])
        h=int((det["bbox_array"][3]-det["bbox_array"][2])*imageshape[1])
        return (x,y,w,h)

    def update_with_detection(self,det,image):
        self.last_bbox=self.get_det_bbox(det,image.shape)
        self.kf.update([self.last_bbox[0],self.last_bbox[1]])
        self.centroid=(int(self.kf.x[0]),int(self.kf.x[1]))
        #self.centroid=(self.last_bbox[0],self.last_bbox[1])
        self.cv_tracker=cv2.TrackerKCF_create()
        self.cv_tracker.init(self.image_to_tracker(image),self.bbox_to_tracker(self.last_bbox))
        self.label=det["label"]


class TrackerGyrusTracker:
    def __init__(self,id=None):
        self.missed_frames=0
        #self.cv_tracker=cv2.TrackerMOSSE_create()
        self.cv_tracker=cv2.TrackerKCF_create()
        self.last_bbox=(0,0,0,0)
        self.label=None
        self.last_success=True
        if id is None:
            self.id=uuid.uuid1()
        else:
            self.id=id

    def init_with_detection(self,image,det):
        self.last_bbox=self.get_det_bbox(det,image.shape)
        self.label=det["label"]
        self.cv_tracker.init(image,self.last_bbox)

    def update(self,frame):
        (self.last_success, box) = self.cv_tracker.update(frame)
        if not self.last_success or box==(0,0,0,0):
            self.missed_frames+=1
            self.last_success=False  #sometimes it gives a bad bounding box but reports success.
        else:
            self.missed_frames=0
            self.last_bbox=box

    def get_det_bbox(self,det,imageshape):
        x=int(0.5*(det["bbox_array"][0]+det["bbox_array"][1])*imageshape[1])
        y=int(0.5*(det["bbox_array"][2]+det["bbox_array"][3])*imageshape[0])
        w=int((det["bbox_array"][1]-det["bbox_array"][0])*imageshape[1])
        h=int((det["bbox_array"][3]-det["bbox_array"][2])*imageshape[1])
        return (x,y,w,h)

    def update_with_detection(self,det,image):
        self.cv_tracker=cv2.TrackerKCF_create()
        self.cv_tracker.init(image,self.get_det_bbox(det,image.shape))
        self.label=det["label"]

    def dist_to(self,xy):
        return np.sqrt((xy[0]-self.last_bbox[0])**2+(xy[1]-self.last_bbox[1])**2)

class TrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.trackers=[]

        self.report_spf_count=10
        self.spf_count=0
        self.spf_sum_time=0
        self.max_missed_frames=10



        super().__init__(broker)

    def get_keys(self):
        return ["image"]

    def get_name(self):
        return "TrackerGyrus"

    def getTracks(self):
        ret=[]
        for tracker in self.trackers:
            mess={}
            bb=tracker.last_bbox
            mess["bbox_array"]=[ int(bb[0]-bb[2]/2),int(bb[0]+bb[2]/2),int(bb[1]-bb[3]/2),int(bb[1]+bb[3]/2)]
            mess["label"]=tracker.label
            mess["id"]=tracker.id
            ret.append(mess)
        return ret

    def update_trackers(self,frame):
        dead_trackers=[]
        for tracker in self.trackers:
            #logger.debug("Updating tracker for {}-{}".format(tracker.label,id_to_name(tracker.id)))
            #logger.debug("from: {}".format(tracker.last_bbox))
            tracker.update(frame)
            if tracker.last_success:
                ...
                #logger.debug("to: {}".format(tracker.last_bbox))
            else:
                #logger.debug("(LOST) to: {}".format(tracker.last_bbox))
                logger.debug("{} is lost! ".format(id_to_name(tracker.id)))
                if tracker.missed_frames>self.max_missed_frames:
                    dead_trackers.append(tracker)
        for tracker in dead_trackers:
            self.trackers.remove(tracker)

    def get_match_score(self,tracker,det,imageshape):
        #so what's important?  Distance match, size match, and label match
        #metric: if you are halfway across the screen and the wrong label, let's call that a mismatch, even if same size
        #that's a distance of like 200 pixels.
        wrong_label_cost=0.3
        dist_cost_weight=0.7/200
        size_cost_weight=0.2/200
        label_cost=0
        if tracker.label!=det["label"]:
            label_cost=wrong_label_cost
        det_bbox=tracker.get_det_bbox(det,imageshape)
        dist_cost=dist_cost_weight*np.sqrt( (det_bbox[0]-tracker.last_bbox[0])**2+(det_bbox[1]-tracker.last_bbox[1])**2)
        size_cost=size_cost_weight*np.sqrt( (det_bbox[2]-tracker.last_bbox[2])**2+(det_bbox[3]-tracker.last_bbox[3])**2)
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
                leftover_dets.append(row_ind[i])
                leftover_trackers.append(col_ind[i])
            else:
                self.trackers[col_ind[i]].update_with_detection(dets[row_ind[i]],image)
        #deal with leftover detections
        for i in range(len(leftover_dets)):
            #new_track=TrackerGyrusTracker()
            new_track=TrackerGyrusTrackerCentroid()
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
        #TODO handle expected changes in motion from gyros
        #So I'd get an expected new angle and can presume how to adjust all my bboxes
        #but how do I convince the tracker to update its bbox?
        if "image" in message and "tracks" not in message:
            #logger.debug("Image Recieved")
            start_time=time.time()
            #first, update all existing trackers
            self.update_trackers(message["image"])

            if "detections" in message:
                self.match_trackers_to_detections(message["detections"],message["image"])

            #TODO reemit message with tracker boxs
            message_out={"image": message["image"],"tracks": self.getTracks(),"timestamp": time.time()}
            #logger.debug("publishing message about tracks")
            self.broker.publish(message_out,["tracks"])
            #timing
            self.spf_count+=1
            self.spf_sum_time+=time.time()-start_time
            if self.spf_count>=self.report_spf_count:
                logging.info("Tracking time fer frame {} ms".format(1000*self.spf_sum_time/self.spf_count))
                self.spf_sum_time=0
                self.spf_count=0
                for tracker in self.trackers:
                    logging.debug("Track: {} is a {} at [{},{}] with velocity [{},{}]".format(id_to_name(tracker.id),tracker.label,tracker.kf.x[0],tracker.kf.x[1],tracker.kf.x[2],tracker.kf.x[3]))
