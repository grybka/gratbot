from Gyrus import ThreadedGyrus
import logging
import uuid
import time
import numpy as np
from underpinnings.ObjectMemory import ObjectMemory

import face_recognition
from underpinnings.FeatureEncoder import image_label_to_vector


logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class ObjectRecognizerGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.memory=ObjectMemory()
        self.memory.load_face_encodings()

        # store most recent track status, unprocessed
        self.recent_tracks={}

        # Maps current tracks to known objects
        self.track_object_map={}

        self.new_object_threshhold=1.0
        #for timing
        self.check_every=1.0 #seconds
        self.last_check=0

    def get_keys(self):
        return ["tracks"]

    def get_name(self):
        return "ObjectRecognizerGyrus"

    def read_message(self,message):
        if 'tracks' in message:
            for track in message["tracks"]:
                self.recent_tracks[track["id"]]=track
            if time.time()>self.last_check+self.check_every:
                self.update_from_tracks(self.recent_tracks)
                self.last_check=time.time()
            else:
                #logger.debug("skipping")
                ...


    def new_track(self,track):
        features={"position": self.track_to_position(track),
                      "image_label": track["label"],
                      "image": track["subimage"]}
        object_id_and_scores=self.memory.identify_object_from_features(features)
        logger.debug(object_id_and_scores)
        if len(object_id_and_scores)>0:
            the_pair=object_id_and_scores[0]
            if the_pair[0]>1.:
                track_object_map[track["id"]]=the_pair[0]
                #logger.debug("match found")
                track["object_id"]=the_pair[0]
                return

        #no object found
        #TODO  figure out if I should instantiate_new_object
        logger.debug("no match found, making new")
        id=self.memory.instantiate_new_object(features)
        self.track_object_map[track["id"]]=id

    def encode_tracks_to_features(self,tracks):
        feature_vecs=[]
        for track in tracks:
            fvec={}
            fvec["track_id"]=track["id"]
            if "label" in track:
                fvec["label"]=track["label"]
                fvec["image_label_vector"]=image_label_to_vector(track["label"])
                if track["label"]=="face" and "subimage" in track:
                    subimage=track["subimage"]
                    bbox=(0,0,subimage.shape[0],subimage.shape[1])
                    fvec["face_encoding"] = face_recognition.face_encodings(subimage,[bbox])[0]
            #position in eye
            bb=track["bbox_array"]
            fvec["xywh"]=[0.5*(bb[0]+bb[1]),0.5*(bb[2]+bb[3]),bb[1]-bb[0],bb[3]-bb[2]]

            feature_vecs.append(fvec)
        return feature_vecs


    def update_from_tracks(self,tracks):
        #separate tracks into ones that are already associated and ones that are new
        unassociated_tracks=[]
        associated_tracks=[]
        for id,track in tracks.items():
            if track["id"] in self.track_object_map:
                if track["info"]=="LOST" or track["info"]=="EXITED":
                    self.memory.track_lost(self.track_object_map[track["id"]])
                else:
                    associated_tracks.append(track)
            else:
                logger.debug("unassoc {}".format(track["label"]))
                #logger.debug("bbox {}".format(track["bbox_array"]))
                unassociated_tracks.append(track)

        #handle associated tracks
        for track in associated_tracks:
            logger.debug("ignoring a {}, already associated".format(track["label"]))
        #encoded_tracks=self.encode_tracks_to_features(associated_tracks)
        #for track in encoded_tracks:
        #    self.memory.update_object_from_feature(track,self.memory.objects[self.track_object_map[track["track_id"]]])

        #handle unassociated_tracks
        encoded_tracks=self.encode_tracks_to_features(unassociated_tracks)
        for track in encoded_tracks:
            #match scores to existing objects
            ids,scores=self.memory.get_object_match_scores(track)
            logger.debug("scores {}".format(scores))
            if len(scores)==0 or np.min(scores)>self.new_object_threshhold:
                logger.debug("new object {}".format(track["label"]))
                #create new object
                id=self.memory.instantiate_new_object(track)
                self.track_object_map[track["track_id"]]=id
            else:
                logger.debug("associating {} with existing object".format(track["label"]))
                i=np.argmin(scores)
                self.track_object_map[track["track_id"]]=ids[i]
                self.memory.update_object_from_feature(track,self.memory.objects[ids[i]])
        #TODO decide if something needs attention
        #publish the annotated tracks
        #self.broker.publish({"timestamp": time.time(),"annotated_tracks": annotated_tracks},["annotated_tracks"])
