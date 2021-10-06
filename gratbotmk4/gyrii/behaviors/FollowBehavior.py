from gyrii.behaviors.Behavior import *
from gyrii.behaviors.FocusBehavior import *
import time
from underpinnings.id_to_name import id_to_name
import logging
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


#brainstorming:
#   TrackFollowerGyrus:  given a visible track, orient myself to it
#   LocalMapSomethingGyrus:   given a position in my local map, orient myself to it
#          -so I need a local map?

#  So a behavior to find someone I might look like
#        -if the person is in view, engage trackfallower gyrus
#        -if the person is on my local map, engage localmapsomethingyrus
#        -failing all that, engage in some sort of search



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

class RunServo(GratbotBehavior):
    def __init__(self,servo_num,servo_angle):
        self.servo_num=servo_num
        self.servo_angle=servo_angle
    def act(self,**kwargs):
        broker=kwargs["broker"]
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"angle": self.servo_angle}}
        broker.publish(servo_command,"servo_command")
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
                message["gyrus_config"]["follow_distance"]=1.0
                message["gyrus_config"]["follow_distance_allowance"]=0.1
            if track["label"]=="face":
                message["gyrus_config"]["follow_distance"]=1.5
                message["gyrus_config"]["follow_distance_allowance"]=0.3

        broker.publish(message,"gyrus_config")
        return GratbotBehaviorStatus.COMPLETED,{}


class SetTailMood(GratbotBehavior):
    def __init__(self,mood):
        self.mood=mood
    def act(self,**kwargs):
        broker=kwargs["broker"]
        message={"timestamp":time.time(),"gyrus_config":{"target_gyrus":"TailGyrus",
                                                         "tail_mood": self.mood}}
        broker.publish(message,"gyrus_config")
        return GratbotBehaviorStatus.COMPLETED,{}

def tail_test():
    return GratbotBehavior_Series([GratbotBehavior_Checklist([SetTailMood("disappointment"),GratbotBehavior_Wait(5.0),
                                                      SetTailMood("anticipation"),GratbotBehavior_Wait(5.0),
                                                      SetTailMood("happiness"),GratbotBehavior_Wait(5.0)])])



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
    return GratbotBehavior_Fallback([FocusOnObjectOfLabelOrdered(allowed_labels),GratbotBehavior_Checklist([SetTailMood("disappointment"),RunMotors(0.5,-0.5,0.3),GratbotBehavior_Wait(1.0)])])

def do_follow():
    return TrackObjectId()
    #return GratbotBehavior_Series([Announce("turning tracking on"),TrackObjectId()])

def find_and_follow(allowed_labels):
    return GratbotBehavior_Series([turn_search(allowed_labels),SetTailMood("anticipation"),do_follow()])

def look_around():
    #turn off tracking
    #look low, left, up, right, down, back
    up_angle=90
    horiz_angle=120
    return GratbotBehavior_Checklist([
            GratbotBehavior_Broadcast({"gyrus_config":{"target_gyrus":"FollowerGyrus","mode": "off"}},"gyrus_config"),
            GratbotBehavior_Broadcast({"gyrus_config":{"target_gyrus":"HeadTrackerGyrus","mode": "off"}},"gyrus_config"),
            GratbotBehavior_Wait(0.5),
            RunServo(0,up_angle),
            GratbotBehavior_Wait(0.5),
            RunMotors(0.5,-0.5,0.5),
            GratbotBehavior_Wait(0.7),
            RunServo(0,horiz_angle),
            GratbotBehavior_Wait(0.5),
            RunMotors(-0.5,0.5,1.0),
            GratbotBehavior_Wait(1.2),
            RunServo(0,up_angle),
            GratbotBehavior_Wait(0.5),
            RunMotors(0.5,-0.5,0.5),
            GratbotBehavior_Wait(0.7)
        ])

class Abehavior(GratbotBehavior):
    def __init__(self):
        ...
    def act(self,**kwargs):
        return GratbotBehaviorStatus.COMPLETED,{}
