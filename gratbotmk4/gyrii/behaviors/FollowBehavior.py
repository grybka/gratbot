from gyrii.behaviors.Behavior import *
from gyrii.behaviors.FocusBehavior import *
import time
from underpinnings.id_to_name import id_to_name
import logging
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class RunMotors(GratbotBehavior):
    def __init__(self,lmotor,rmotor,duration):
        self.lmotor=lmotor
        self.rmotor=rmotor
        self.duration=duration
    def act(self,**kwargs):
        broker=kwargs["broker"]
        motor_command={"timestamp": time.time(),"motor_command": {"left_throttle": self.lmotor,
                                                                   "right_throttle": self.rmotor,
                                                                   "left_duration": self.duration,
                                                                   "right_duration": self.duration},"keys": ["motor_command"]}
        #logging.debug("Emitting Motor Command {}".format(motor_command))
        broker.publish(motor_command,"motor_command")
        return GratbotBehaviorStatus.COMPLETED,{}

def extract_track_with_id(short_term_memory,id):
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["id"]==id:
                return track
    return None

class TrackObjectId(GratbotBehavior):
    def __init__(self):
        ...
    def act(self,**kwargs):
        broker=kwargs["broker"]
        message={"timestamp":time.time(),"gyrus_config":{"target_gyrus":"FollowerGyrus",
                                                         "tracked_object":kwargs["focus_track_id"],
                                                         "mode": "track_target"}}
        track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
        if track is not None:
            if track["label"]=="person":
                message["gyrus_config"]["follow_distance"]=1.8
                message["gyrus_config"]["follow_distance_allowance"]=0.1
            if track["label"]=="face":
                message["gyrus_config"]["follow_distance"]=0.5
                message["gyrus_config"]["follow_distance_allowance"]=0.3

        broker.publish(message,"gyrus_config")
        return GratbotBehaviorStatus.COMPLETED,{}




#Look around for someone/something
#if I find something, follow it
#if someone isn't moving much, go somewhere else
#

#SEARCH
#this will not involve wandering until I have some way of not hitting stuff

#Turn to preferred side
#returns true if I've found something
#in progress if I haven't
def turn_search(allowed_labels):
    return GratbotBehavior_Fallback([FocusOnObjectOfLabelOrdered(allowed_labels),GratbotBehavior_Checklist([Announce("Nothing found, turning"),RunMotors(0.5,-0.5,0.1),GratbotBehavior_Wait(0.4)])])

def do_follow():
    return TrackObjectId()
    #return GratbotBehavior_Series([Announce("turning tracking on"),TrackObjectId()])

def find_and_follow(allowed_labels):
    return GratbotBehavior_Series([turn_search(allowed_labels),do_follow()])



class Abehavior(GratbotBehavior):
    def __init__(self):
        ...
    def act(self,**kwargs):
        return GratbotBehaviorStatus.COMPLETED
