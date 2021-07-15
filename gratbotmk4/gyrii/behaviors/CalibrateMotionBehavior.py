from gyrii.behaviors.Behavior import *
import time
import numpy as np
from math import sin,cos
import random
import logging
from underpinnings.id_to_name import id_to_name

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
        #logging.info("Emitting Motor Command {}".format(motor_command))
        broker.publish(motor_command,"motor_command")
        return GratbotBehaviorStatus.COMPLETED

class TurnRelative(GratbotBehavior):
    def __init__(self,angle):
        self.angle=angle #in radians
    def act(self,**kwargs):
        broker=kwargs["broker"]
        motion_command={"timestamp": time.time(),"motion_command": {"category": "turn_relative",
                                                                    "radians": self.angle}}
        broker.publish(motion_command,"motion_command")
        return GratbotBehaviorStatus.COMPLETED

class ExerciseTurns(GratbotBehavior):
    def __init__(self):
        motion_list=[]
        turnmotion=np.linspace(np.pi/32,np.pi/2,4)
        for m in turnmotion:
            motion_list.append(self.wrap_motion_act(TurnRelative(m)))
            motion_list.append(self.wrap_motion_act(TurnRelative(-m)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehaviorFB(GratbotBehavior):

    def __init__(self):
        nsteps=6
        motion_list=[]
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[1.0]*nsteps
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehavior(GratbotBehavior):
    def __init__(self):
        nsteps=6
        motion_list=[]
        #aheadmotion=np.linspace(0.2,1.0,nsteps)
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[0.2]*nsteps
#        for i in range(nsteps): #fb
#            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
#            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],-aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],aheadmotion[i],aheaddur[i])))
        nsteps=5
        #aheadmotion=np.linspace(0.3,1.0,nsteps)
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[0.2]*nsteps
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],0.5*aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-0.5*aheadmotion[i],aheaddur[i])))
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(0.5*aheadmotion[i],aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-0.5*aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #aheaddur=[0.2]*nsteps
        #for i in range(nsteps): #fb
        #    motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
        #    motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #for i in range(nsteps): #turn
        #    motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #    motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],aheadmotion[i],aheaddur[i])))

        #ahead_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in ahead_motion:
        #    motion_list.append(self.wrap_motion_act(MoveAhead(am)))
        #turn_motion=[0.2,-0.2,0.4,-0.4,0.6,-0.6,0.8,-0.8]
        #for i in range(100):
        #     motion_list.append(self.wrap_motion_act(Turn(random.choice(turn_motion))))

        #for am in turn_motion:
        #    motion_list.append(self.wrap_motion_act(Turn(am)))
        #slew_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in slew_motion:
        #    motion_list.append(self.wrap_motion_act(Slew(am)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )
        #return GratbotBehavior_Series([WaitForStablePose(),Announce("moving"),motion_act,GratbotBehavior_Wait(2.0),WaitForStablePose()])

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehavior_WithTracking_Turns(GratbotBehavior):
    def __init__(self,labels):
        self.to_track=labels
        self.sub_behavior=None
        self.next_timestamp=0
        self.power_dur_product=0.15
        self.things_to_try=[ 0.4,0.5,0.6,0.7,0.8,0.9,1.0]
        #self.things_to_try=[ [0.4,0.5],[0.6,0.4],[0.7,0.3],[0.8,0.2]]
        self.on_thing_to_try=0

    def act(self,**kwargs):
        #if I'm in the middle of something, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.INPROGRESS:
                return step_status
            else:
                self.sub_behavior=None
            return GratbotBehaviorStatus.INPROGRESS

        #time to figure out something new to do
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            the_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            if the_timestamp<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info

            for track in tracks:
                logger.info("examining {}".format(track["label"]))
                if track["label"] in self.to_track:
                    #I presume its holding still here
                    error=track["center"][0]-0.5
                    #if I'm too far away from centered, recenter on object
                    if abs(error)>0.1:
                        logger.info("correcting")
                        x=np.sign(error)*0.4
                        todo=RunMotors(x,-x,0.1)
                        self.next_timestamp=the_timestamp+0.2
                        todo.act(**kwargs)
                        return GratbotBehaviorStatus.INPROGRESS
                    #Now try one of those other actions
                    elem=self.things_to_try[self.on_thing_to_try]
                    dur=self.power_dur_product/elem
                    logger.info("taking a turn {} , {}".format(elem,dur))
                    self.sub_behavior=self.wrap_motion_act(RunMotors(elem,-elem,dur),RunMotors(-elem,elem,dur))
                    self.on_thing_to_try+=1
                    if self.on_thing_to_try>=len(self.things_to_try):
                        self.on_thing_to_try=0
                    return GratbotBehaviorStatus.INPROGRESS
                logger.info("ignoring {}".format(track["label"]))

        #if I can't find the object I'm tracking, just wait
        logger.info("Can't find what I'm looking for.  Waiting. ")
        return GratbotBehaviorStatus.INPROGRESS

    def wrap_motion_act(self,motion_act1,motion_act2):
        return GratbotBehavior_Series([motion_act1,GratbotBehavior_Wait(1.0),motion_act2,GratbotBehavior_Wait(1.0)] )



class CalibrateMotionBehavior_WithTracking_FB(GratbotBehavior):
    def __init__(self,labels):
        self.to_track=labels
        self.sub_behavior=None
        self.next_timestamp=0
        self.power_dur_product=1.2
        self.things_to_try=[ 0.4,0.5,0.6,0.7,0.8,0.9,1.0]
        #self.things_to_try=[ [0.4,0.5],[0.6,0.4],[0.7,0.3],[0.8,0.2]]
        self.on_thing_to_try=0

    def act(self,**kwargs):
        #if I'm in the middle of something, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.INPROGRESS:
                return step_status
            else:
                self.sub_behavior=None
            return GratbotBehaviorStatus.INPROGRESS

        #time to figure out something new to do
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            the_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            if the_timestamp<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info

            for track in tracks:
                logger.info("examining {}".format(track["label"]))
                if track["label"] in self.to_track:
                    #I presume its holding still here
                    error=track["center"][0]-0.5
                    #if I'm too far away from centered, recenter on object
                    if abs(error)>0.1:
                        logger.info("correcting")
                        x=np.sign(error)*0.4
                        todo=RunMotors(x,-x,0.1)
                        self.next_timestamp=the_timestamp+0.2
                        todo.act(**kwargs)
                        return GratbotBehaviorStatus.INPROGRESS
                    #Now try one of those other actions
                    elem=self.things_to_try[self.on_thing_to_try]
                    dur=self.power_dur_product/elem
                    logger.info("taking a fb  {} , {}".format(elem,dur))
                    self.sub_behavior=self.wrap_motion_act(RunMotors(elem,elem,dur),RunMotors(-elem,-elem,dur),dur)
                    self.on_thing_to_try+=1
                    if self.on_thing_to_try>=len(self.things_to_try):
                        self.on_thing_to_try=0
                    return GratbotBehaviorStatus.INPROGRESS
                logger.info("ignoring {}".format(track["label"]))

        #if I can't find the object I'm tracking, just wait
        logger.info("Can't find what I'm looking for.  Waiting. ")
        return GratbotBehaviorStatus.INPROGRESS

    def wrap_motion_act(self,motion_act1,motion_act2,extra_wait):
        return GratbotBehavior_Series([motion_act1,GratbotBehavior_Wait(1.0+extra_wait),motion_act2,GratbotBehavior_Wait(1.0+extra_wait)] )
