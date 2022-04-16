#TODO Periodically look at tracks that are labelled faces and see if I know any of them
#TODO Emit an association message if I do
#TODO collect examples of known faces to improve learning
#TODO how to deal with learning new faces?

import logging
import time
import uuid
import face_recognition
from Gyrus import ThreadedGyrus
from collections import deque
import cv2
import numpy as np
import yaml

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class FaceRecognizer(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display# for debug display
        super().__init__(broker)

        self.clear_subimage_map()
        self.last_images=deque([],5) #maxlength is 3, entries are [timestamp,image]
        self.report_every_seconds=2 #how often to run the calculation
        self.last_report_time=time.time()

        self.face_dictionary={} #maps ID to vectors
        self.name_dictionary={} #maps ID to name
        self.unknown_face_dictionary={} #maps ID to vectors
        self.unknown_face_image_dictionary={} #maps ID to vectors
        self.max_unknown_faces_per_run=5
        self.match_tolerance=0.6
        self.load_face_dictionary()

    def get_keys(self):
        return ["image","tracks","clock_pulse"]

    def get_name(self):
        return "FaceRecognizer"

    def clear_subimage_map(self):
        self.track_subimage_map={}

    def load_face_dictionary(self):
        config_filename="config/face_knowledge.yaml"
        with open(config_filename,'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            for elem in data:
                id=uuid.uuid1()
                logger.debug("Loading faces for {}".format(elem["name"]))
                self.name_dictionary[id]=elem["name"]
                self.face_dictionary[id]=[]
                for fname in elem["face_images"]:
                    logger.debug("loading {}".format(fname))
                    try:
                        subimage = cv2.imread(fname)
                        bbox=(0,0,subimage.shape[0],subimage.shape[1])
                        face_encoding = face_recognition.face_encodings(subimage,[bbox])[0]
                        self.face_dictionary[id].append(face_encoding)
                    except:
                        logger.error("Unable to fopen file {}".format(fname))

    def match_face(self,encoding,facedict):
        ids=[]
        dists=[]
        for id in facedict:
            for face in facedict[id]:
                ids.append(id)
                dists.append(np.linalg.norm(face-encoding))
        if len(ids)==0:
            return [],[]

        return zip(*sorted(zip(dists,ids))) #dists, ids

    def match_seen_face(self,subimage):
        bbox=(0,0,subimage.shape[0],subimage.shape[1])
        face_encoding = face_recognition.face_encodings(subimage,[bbox])[0]
        dists,ids=self.match_face(face_encoding,self.face_dictionary)
        if len(dists)!=0 and dists[0]<self.match_tolerance: #counts as a match
            #TODO should I then give a prob distribution
            logger.debug("Matched face to {} ({})".format(self.name_dictionary[ids[0]],dists[0]))
            return ids[0]
        else:
            dists,ids=self.match_face(face_encoding,self.unknown_face_dictionary)
            #logger.debug("matches: {} {}".format(dists,ids))
            if len(dists)!=0 and dists[0]<self.match_tolerance: #already have this face
                logger.debug("Already have this unknown face")
                return None
            else:
                if len(ids)>=self.max_unknown_faces_per_run:
                    logger.debug("max unknown faces hit, not saving")
                    return None
                logger.debug("recording new unknown_face")
                #generate a new id
                new_id=uuid.uuid1()
                self.unknown_face_dictionary[new_id]=[face_encoding]
                self.unknown_face_image_dictionary[new_id]=[subimage]
                fname="logs/unkown_face_{}.png".format(len(ids))
                cv2.imwrite(fname,subimage)
                return None


    def update_tracks(self,message):
        for track in message["tracks"]:
            the_image=None
            for ipairs in self.last_images:
                if ipairs[0]==message["image_timestamp"]:
                    the_image=ipairs[1]
            if the_image is None:
                #TODO this happens a lot.  How to fix?
                logger.debug("couldn't find an image to match track timestamp: {}, available:".format(message["image_timestamp"]))
                for ipairs in self.last_images:
                    logger.debug("{}".format(ipairs[0]))
            else:
                #only take the first of each instance
                if track["id"] not in self.track_subimage_map:
                    x1=int(track["last_detection_bbox"][0]*the_image.shape[1])
                    x2=int(track["last_detection_bbox"][1]*the_image.shape[1])
                    y1=int(track["last_detection_bbox"][2]*the_image.shape[0])
                    y2=int(track["last_detection_bbox"][3]*the_image.shape[0])
                    self.track_subimage_map[track["id"]]=the_image[y1:y2,x1:x2]

    def clock_pulse(self):
        if time.time()>self.last_report_time+self.report_every_seconds:
            reports=[]
            for track_id in self.track_subimage_map:
                #run the id software here

                #TODO I guess I want to do matching here?
                face_id=self.match_seen_face(self.track_subimage_map[track_id])

                #TODO I should probably limit how many I try at once

                if face_id==None:
                    id_report={"track_id": track_id,"face_id": "UNKNOWN", "name": "UNKNOWN"}
                else:
                    id_report={"track_id": track_id,"face_id": face_id, "name": self.name_dictionary[face_id]}
                reports.append(id_report)
            message_out={"track_identifications": reports,"timestamp": time.time()}
            self.broker.publish(message_out,["track_identifications"])
            self.last_report_time=time.time()
            if self.display is not None:
                if len(self.track_subimage_map)!=0:
                    an_index=next(iter(self.track_subimage_map))
                    self.display.update_image("face",self.track_subimage_map[an_index])
            self.clear_subimage_map()

    def read_message(self,message):
        if "image" in message:
            self.last_images.append([message["image_timestamp"],message["image"]])
        if "tracks" in message:
            self.update_tracks(message)
        if "clock_pulse" in message:
            self.clock_pulse()
