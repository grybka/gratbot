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

    def update_from_tracks(self,tracks):
        track_object_map={}
        for track in tracks:
            features={"position": self.track_to_position(track),
                      "image_label": track["label"],
                      "image": track["subimage"]}
            object_id_and_scores=self.memory.identify_object_from_features(features)
            logger.debug(object_id_and_scores)
            if len(object_id_and_scores)>0:
                the_pair=object_id_and_scores[0]
                if the_pair[0]>1.:
                    track_object_map[track["id"]]=the_pair[0]
                    logger.debug("match found")
                    continue
            #no object found
            logger.debug("no match found")

        #TODO decide if something needs attention
        #self.broker.publish({"timestamp": time.time(),"object_attention": rec},["attention"])


    def track_to_position(self,track):
        ...
