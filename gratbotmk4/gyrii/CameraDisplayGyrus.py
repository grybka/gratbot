
from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
import logging,time
import numpy as np
import cv2

logger=logging.getLogger(__name__)
#logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

class TrackMerger:
    def __init__(self):
        self.tracks={}

    def merge_tracks(self,message):
        for t in message["tracks"]:
            if t["info"]=="LOST":
                self.tracks.pop(t["id"],None)
            else:
                self.tracks[t["id"]]=t
                self.tracks[t["id"]]["last_update"]=time.time()
        to_drop=[]
        for id in self.tracks:
            if time.time()-self.tracks[id]["last_update"]>10:
                to_drop.append(id)
        for id in to_drop:
            self.tracks.pop(id,None)

class CameraDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None,mode=None):
        self.display=display
        self.show_fps=True
        self.fps=0
        self.fps_count=11
        self.fps_count_reset=10
        self.fps_start_time=0

        self.display_subimages=False

        if mode==None:
            self.mode="show_detections"
        else:
            self.mode=mode
        #self.mode="show_tracks"
        super().__init__(broker)

        self.tracks={}
        self.detections={}



        self.last_detections_message={"detections": []}
        self.last_image_message={}
        self.last_depth_message={}

    def get_keys(self):
        return ["image","tracks","depth","test_image","detections"]

    def get_name(self):
        return "CameraDisplayGyrus"

    def update_fps_and_put_text(self,frame):
        self.fps_count+=1
        if self.fps_count>=self.fps_count_reset:
            self.fps=self.fps_count_reset/(abs(time.time()-self.fps_start_time)+1e-3)
            self.fps_start_time=time.time()
            self.fps_count=0
        #frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #print("frame type {}".format(frame.dtype))
        #print("frame shape {}".format(frame.shape))
        color = (255, 0, 0)
        cv2.putText(frame, "NN fps: {:.2f}".format(self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

    def draw_detection_bbox(self,frame,d):
        color = (255, 0, 0)
        height = frame.shape[0]
        width  = frame.shape[1]
        try:
            x1 = int(d["bbox_array"][0] *width )
            x2 = int(d["bbox_array"][1] *width)
            y1 = int(d["bbox_array"][2] *height)
            y2 = int(d["bbox_array"][3] *height)

            ##x1 = int(d["bbox_array"][0] * width)
            #x2 = int(d["bbox_array"][1] * width)
            #y1 = int(d["bbox_array"][2] * height)
            #y2 = int(d["bbox_array"][3] * height)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
            cv2.putText(frame, str(d["label"]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        except:
            logger.warning("failed to display deteciton {}".format(d))
        if "confidence" in d:
            cv2.putText(frame, "{:.2f}".format(d["confidence"]*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        if "spatial_array" in d:
            cv2.putText(frame, f"X: {int(d['spatial_array'][0])} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(d['spatial_array'][1])} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(d['spatial_array'][2])} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

    def draw_track_bbox(self,frame,t):
        color = (255, 0, 0)
        height = frame.shape[0]
        width  = frame.shape[1]
        if t["info"]=="PROBATION":
            return
        x1 = int(t["bbox_array"][0]*width)
        x2 = int(t["bbox_array"][1]*width)
        y1 = int(t["bbox_array"][2]*height)
        y2 = int(t["bbox_array"][3]*height)

        #x1 = int(t["bbox_array"][0]*width )
        #x2 = int(t["bbox_array"][1]*width)
        #y1 = int(t["bbox_array"][2]*height )
        #y2 = int(t["bbox_array"][3]*height )
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
        #cv2.putText(frame, str(id_to_name(t["id"])), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, "{} ({})".format(id_to_name(t["id"]),t["label"]), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        #cv2.putText(frame, "{:.2f} {:.2f} Dist: {:.2f}m".format(t["center"][0],t["center"][1],t["center"][2]), (x1+10,y1+35),cv2.FONT_HERSHEY_TRIPLEX,0.5,255)
        cv2.putText(frame, "{}".format(t["info"]), (x1+10,y1+35),cv2.FONT_HERSHEY_TRIPLEX,0.5,255)

    def update_display(self):
        if "image" not in self.last_image_message:
            return
        #logger.debug("saw image")
        frame=np.copy(self.last_image_message["image"])
        #logger.debug("frame size {}".format(frame.shape))
        if self.mode=="show_detections":
            self.max_detection_lag=0.5
            for detection_type in self.detections:
                lag=-self.detections[detection_type]["image_timestamp"]+self.last_image_message["image_timestamp"]
                if lag<self.max_detection_lag:
                    for d in self.detections[detection_type]["detections"]:
                        self.draw_detection_bbox(frame,d)
                #else:
                #    logger.debug("{} lag too great {} ms".format(detection_type,1000*lag))
        elif self.mode=="show_tracks":
            self.max_track_lag=0.5
            for track_type in self.tracks:
                lag=-self.tracks[track_type]["image_timestamp"]+self.last_image_message["image_timestamp"]
                if lag<self.max_track_lag:
                    for t in self.tracks[track_type]["tracks"]:
                        self.draw_track_bbox(frame,t)
                else:
                    logger.debug("{} lag too great {} ms".format(track_type,1000*lag))
        if self.show_fps==True:
            self.update_fps_and_put_text(frame)
        self.display.update_image("camera",frame)

    def update_depth(self):
        if "depth_image" in self.last_depth_message:
            frame=self.last_depth_message["depth_image"]
            frame=(frame*(255./95.)).astype(np.uint8)
            #myframe=(255*frame/3e3).astype(np.uint8)
            #cv2.putText(frame, "{} x {}".format(frame.shape[0],frame.shape[1]))
            self.display.update_image("depth",frame)

    def read_message(self,message):
        if "image" in message:
            self.last_image_message=message
            #print("image timestamp {}".format(message["image_timestamp"]))
            self.update_display()
        if "detections" in message:
            #print("detection timestamp {}".format(message["image_timestamp"]))
            #logger.debug("got detections {}".format(message))
            self.detections[message["detection_name"]]=message

            if self.display_subimages and len(message["detections"])>0:
                if "subimage" in message["detections"][0] and self.mode=="show_detections":
                    self.display.update_image("detsubimage",message["detections"][0]["subimage"])
        if "tracks" in message and self.mode=="show_tracks":
            self.tracks[message["detection_name"]]=message
            #self.last_tracks_message=message
            #if self.display_subimages and len(message["tracks"])>0:
            #    self.display.update_image("subimage",message["tracks"][0]["subimage"])

        if "depth_image" in message:
            self.last_depth_message=message
            self.update_depth()
        if "test_image" in message:
            frame=message["test_image"]
            self.display.update_image("test",frame)
