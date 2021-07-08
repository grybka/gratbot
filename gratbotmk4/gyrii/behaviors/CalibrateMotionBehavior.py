from Behavior import *
from Behavior import GratbotBehaviorStatus
from MotionBehaviors import *
import time
import numpy as np
from math import sin,cos
from underpinnings.BayesianArray import BayesianArray
import random


class RunMotors(GratbotBehavior):
    def __init__(self,lmotor,rmotor,duration):
        self.lmotor=lmotor
        self.rmotor=rmotor
        self.duration=duration
    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"left_throttle": self.lmotor,
                                                                   "right_throttle": self.rmotor,
                                                                   "left_duration": self.duration,
                                                                   "right_duration": self.duration},"motor_command")
        return GratbotBehaviorStatus.COMPLETED

class CalibrateMotionBehavior(GratbotBehavior):
    def __init__(self):
        nsteps=15
        motion_list=[]
        aheadmotion=np.linspace(0.1,1.0,nsteps)
        aheaddur=[0.2]*nsteps
        for i in range(nsteps): #fb
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],-aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],aheadmotion[i],aheaddur[i])))
        nsteps=5
        aheadmotion=np.linspace(0.3,1.0,nsteps)
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
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(0.5)] )
        #return GratbotBehavior_Series([WaitForStablePose(),Announce("moving"),motion_act,GratbotBehavior_Wait(2.0),WaitForStablePose()])

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)
