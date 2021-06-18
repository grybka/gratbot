
from Behavior import *
from Behavior import GratbotBehaviorStatus
from MotionBehaviors import *
import time
import numpy as np
from math import sin,cos
from underpinnings.BayesianArray import BayesianArray
import random

class CalibrateMotionBehavior(GratbotBehavior):
    def __init__(self):
        motion_list=[]
        #ahead_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in ahead_motion:
        #    motion_list.append(self.wrap_motion_act(MoveAhead(am)))
        turn_motion=[0.2,-0.2,0.4,-0.4,0.6,-0.6,0.8,-0.8]
        for i in range(100):
             motion_list.append(self.wrap_motion_act(Turn(random.choice(turn_motion))))

        #for am in turn_motion:
        #    motion_list.append(self.wrap_motion_act(Turn(am)))
        #slew_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in slew_motion:
        #    motion_list.append(self.wrap_motion_act(Slew(am)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([WaitForStablePose(),Announce("moving"),motion_act,GratbotBehavior_Wait(2.0),WaitForStablePose()])

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class ComplexMotion(GratbotBehavior):
    def __init__(self,angle,duration):
        self.angle=angle
        self.duration=duration
        self.motor4vector=[ np.cos(self.angle),0,np.sin(self.angle),self.duration ]

    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"type": "translate","vector": self.motor4vector}},"motor_command")
        return GratbotBehaviorStatus.COMPLETED

class RandomMotion(GratbotBehavior):
    def __init__(self):
        self.max_duration=1.0
        self.min_duration=0.05
    def act(self,**kwargs):
        duration=random.uniform(self.min_duration,self.max_duration)
        lt=random.choice([-1, 1])*random.uniform(0.4,1.0)
        rt=random.choice([-1, 1])*random.uniform(0.4,1.0)
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration":duration } },"motor_command")
        return GratbotBehaviorStatus.COMPLETED


class RandomSmoothMotion(GratbotBehavior):
    def __init__(self):
        self.max_duration=0.5
        self.min_duration=0.09
        self.start_time=0
        self.last_time=0
        self.duration=0
        self.action_freq=0.1
    def act(self,**kwargs):
        if self.start_time==0:
            self.start_time=time.time()
            self.duration=random.uniform(self.min_duration,self.max_duration)
            self.lt=random.choice([-1, 1])*random.uniform(0.4,1.0)
            self.rt=random.choice([-1, 1])*random.uniform(0.4,1.0)
        if time.time()>self.start_time+self.duration:
            return GratbotBehaviorStatus.COMPLETED
        if time.time()<self.last_time+self.action_freq:
            return GratbotBehaviorStatus.INPROGRESS
        self.last_time=time.time()
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [self.lt,self.rt], "duration":self.action_freq } },"motor_command")
        return GratbotBehaviorStatus.INPROGRESS


class RandomMotionTrackVisual(GratbotBehavior):
    def __init__(self):
        self.max_duration=0.2
        self.min_duration=0.05
        self.last_motion=None
        self.last_tracked_objects={}
        self.useful_data=[]
        self.sub_behavior=None

    def act(self,**kwargs):
        step_status=GratbotBehaviorStatus.COMPLETED
        if self.sub_behavior is not None:
            #gprint("sub_behavior")
            step_status=self.sub_behavior.act(**kwargs)
        if step_status==GratbotBehaviorStatus.COMPLETED:
            gprint("start")
            self.sub_behavior=GratbotBehavior_Series([Announce("moving"),RandomSmoothMotion(),GratbotBehavior_Wait(0.2),Announce("waiting for motors to stope"),WaitForMotorsOff(),GratbotBehavior_Wait(0.4)])
            #self.record_object_positions(kwargs["short_term_memory"])
            #motion=ComplexMotion(theta,duration)
            #self.last_motion=motion.motor4vector
            #self.sub_behavior=GratbotBehavior_Series([Announce("moving"),motion,GratbotBehavior_Wait(1.0),WaitForStablePose()])
            return GratbotBehaviorStatus.INPROGRESS

    def record_object_positions(self,short_term_memory):
        if "visual_tracker_objects" not in short_term_memory:
            self.last_tracked_objects={}
            return
        visual_tracker_objects=short_term_memory["visual_tracker_objects"]["visual_tracker_objects"]
        #gprint("visual tracker objects {}".format(visual_tracker_objects))
        #gprint("last tracked {}".format(self.last_tracked_objects))
        #gprint("last motion is {}".format(self.last_motion))
        #for each object, if we saw it before, figure out
        if self.last_motion is not None:
            for obj in visual_tracker_objects:
                if obj["id"] in self.last_tracked_objects:
                    delta=np.array(obj["xywh"])-np.array(self.last_tracked_objects[obj["id"]]["xywh"])
                    self.useful_data.append( [self.last_motion,self.last_tracked_objects[obj["id"]]["xywh"],obj["xywh"]])
        self.last_tracked_objects={}
        for obj in visual_tracker_objects:
            self.last_tracked_objects[obj["id"]]=obj
        #temporary for testing in notebooks
        f=open("testfile.txt",'w')
        for obj in self.useful_data:
            f.write("{} {} {} {} {} {} {} {} {} {} {} {}\n".format(*obj[0],*obj[1],*obj[2]))
        f.close()
