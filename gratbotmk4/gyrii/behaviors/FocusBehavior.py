from gyrii.behaviors.Behavior import *
import time
import numpy as np
from math import sin,cos
import random
import logging
from underpinnings.id_to_name import id_to_name

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

def extract_track_with_label(short_term_memory,labels):
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["label"] in labels:
                return track
    return None


    best_choice=[None,None]
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["label"] in labels:
                return track
    return None

def extract_track_with_id(short_term_memory,id):
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["id"]==id:
                return track
    return None

class FocusOnObjectOfLabel(GratbotBehavior):
    def __init__(self,labels):
        self.labels=labels
    def act(self,**kwargs):
        if "focus_track_id" in kwargs["state"]:
            to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
            if to_track["label"] in self.labels:
                return GratbotBehaviorStatus.COMPLETED,{"focus_track_id": to_track["id"]}
            else:
                kwargs["state"]["focus_track_id"]=None
        to_track=extract_track_with_label(kwargs["short_term_memory"],self.labels)
        if to_track is None:
            return GratbotBehaviorStatus.FAILED,{"focus_track_id": None}
        kwargs["state"]["focus_track"]=to_track["id"]
        return GratbotBehaviorStatus.COMPLETED,{"focus_track_id": to_track["id"]}

class FocusOnObjectOfLabelOrdered(GratbotBehavior):
    def __init__(self,labels):
        self.labels=labels
    def act(self,**kwargs):
        short_term_memory=kwargs["short_term_memory"]
        best_choice=None
        if "focus_track_id" in kwargs["state"]:
            to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
            if to_track["label"] in self.labels:
                best_choice=to_track
        if "tracks" in short_term_memory:
            for track in short_term_memory["tracks"]["tracks"]:
                if track["label"] in labels:
                    if best_choice is None or self.labels.index(track["label"])<self.labels.index(best_choice["label"]):
                        logger.debug("choosing to track a {}".format(track["label"]))
                        best_choice=track
        if to_track is None:
            return GratbotBehaviorStatus.FAILED,{"focus_track_id": None}
        kwargs["state"]["focus_track"]=to_track["id"]
        return GratbotBehaviorStatus.COMPLETED,{"focus_track_id": to_track["id"]}
