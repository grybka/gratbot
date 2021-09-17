from Gyrus import ThreadedGyrus
import logging
import uuid
import time
import numpy as np
from underpinnings.ObjectMemory import ObjectMemory
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class ObjectRecognizerGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.memory=ObjectMemory()
        self.memory.load_face_encodings()

        # Maps current tracks to known objects
        self.track_object_map={}

        #for timing
        self.check_every=0.5 #seconds
        self.last_check=0

    def get_keys(self):
        return ["tracks"]

    def get_name(self):
        return "ObjectRecognizerGyrus"

    def read_message(self,message):
        if 'tracks' in message:
            if message["timestamp"]>self.last_check+self.check_every:
                self.update_from_tracks(message["tracks"])
                self.last_check=message["timestamp"]

    def update_track(self,object_id,track):
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
                continue
        #no object found
        #TODO  figure out if I should instantiate_new_object
        logger.debug("no match found, making new")
        id=self.memory.instantiate_new_object(features)
        self.track_object_map[track["id"]]=id

    def update_from_tracks(self,tracks):
        #annotated_tracks=[]
        for track in tracks:
            if track["id"] in self.track_object_map:
                self.update_track(self.track_object_map[track["id"]],track)
            else:
                self.new_track(track)
        #TODO decide if something needs attention
        #publish the annotated tracks
        #self.broker.publish({"timestamp": time.time(),"annotated_tracks": annotated_tracks},["annotated_tracks"])

    def track_to_position(self,track):
        ...
