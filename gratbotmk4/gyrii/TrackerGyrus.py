#using opencv tracker

# will track only one of each type
import cv2
import uuid
from underpinnings.id_to_name import id_to_name

from Gyrus import ThreadedGyrus

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class TrackerGyrusTracker:
    def __init__(self,label,id=None):
        self.alive=True
        self.max_missed_frames=10
        self.missed_frames=0
        self.cv_tracker=cv2.TrackerMOSSE_create()
        self.last_bbox=(0,0,0,0)
        self.label=label
        if id is None:
            self.id=uuid.uuid1()
        else:
            self.id=id

    def init(self,image,bbox):
        self.cv_tracker.init(image,bbox)

    def update(self,frame):
        (success, box) = self.cv_tracker.update(frame)
        if success:
            self.missed_frames=0
            self.last_bbox=box
        else:
            self.missed_frames+=1
        if self.missed_frames>self.max_missed_frames:
            self.alive=False

    def dist_to(self,xy):
        return np.sqrt((xy[0]-self.last_bbox[0])**2+(xy[1]-self.last_bbox[1])**2)**2)**2)**2)

class TrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.trackers={}

        self.report_spf_count=30
        self.spf_count=0
        self.spf_sum_time=0



        super().__init__(broker)

    def get_keys(self):
        return ["image"]

    def get_name(self):
        return "TrackerGyrus"

    def create_new_tracker(self,image,bbox,label):
        self.trackers[label]=TrackerGyrusTracker(label)
        self.trackers[label].init(image,bbox)

    def sort_detections_by_label(self,detections):
        ret={}
        for det in detections:
            if det["label"] in ret:
                ret[det["label"]].append(det)
            else:
                ret[det["label"]]=[det]
        return ret

    def getTracks(self):
        ret=[]
        for key in self.trackers:
            if self.trackers[key].alive:
                mess={}
                bb=self.trackers[key].last_bbox
                mess["bbox_array"]=[ int(bb[0]-bb[2]/2),int(bb[0]+bb[2]/2),int(bb[1]-bb[3]/2),int(bb[1]+bb[3]/2)]
                mess["label"]=self.trackers[key].label
                mess["id"]=self.trackers[key].id
                ret.append(mess)
        return ret


    def read_message(self,message):
        if "image" in message:
            logger.debug("Image Recieved")
            start_time=time.time()
            #first, update all existing trackers
            for tracker in self.trackers:
                if tracker.alive:
                    logger.debug("Updating tracker for {}-{}".format(tracker.label,id_to_name(tracker.uuid)))
                    tracker.update(message["image"])

            #if detections were made, then see if I should match them to existing tracks, or make new trackers
            if "detections" in message:
                dets=self.sort_detections_by_label(message["detections"])
                for label in dets:
                    logger.debug("{} detections of label {}".format(len(dets[label]),label))
                    min_dist=np.inf
                    best_one=None
                    for i in range(len(dets[label])):
                        det=dets[label][i]
                        x=int((0.5*det["bbox_array"][0]+det["bbox_array"][1])*message["image"].shape[1])
                        y=int((0.5*det["bbox_array"][2]+det["bbox_array"][3])*message["image"].shape[0])
                        w=int((det["bbox_array"][1]-det["bbox_array"][0])*message["image"].shape[1])
                        h=int((det["bbox_array"][3]-det["bbox_array"][2])*message["image"].shape[1])
                        if det["label"] in self.trackers and self.trackers[det["label"]].alive:
                            dist=self.trackers[det["label"]].dist_to( (x,y) )
                            logger.debug("dist {}".format(dist))
                            if dist<min_dist:
                                min_dist=dist
                                best_bbox=(x,y,w,h)
                                best_det=det
                        else:
                            logger.debug("creating new tracker with no reference")
                            self.create_new_tracker(det,(x,y,w,h))
                            continue #so I only do the first
                    #here I've found the closest detection to the track I was following
                    #note this just recreates the tracker.  I guess that's OK
                    logger.debug("rebaselining tracker {}".format(id_to_name(self.trackers[det["label"]].id)))
                    self.create_new_tracker(best_det,best_bbox,id=self.trackers[det["label"]].id)
            #TODO reemit message with tracker boxs
            message_out={"image": message["image"],"tracks": self.getTracks(),"timestamp": time.time()}
            self.broker.publish(message_out,["tracks"])
            #timing
            self.spf_count+=1
            self.spf_sum_time+=start_time-time.time()
            if self.spf_count>=self.report_spf_count:
                logging.info("Tracking time fer frame {} ms".format(1000*self.spf_sum_time/self.spf_count))
                self.spf_sum_time=0
                self.spf_count=0
