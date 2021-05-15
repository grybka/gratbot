
from Behavior import *
from Behavior import GratbotBehaviorStatus
from MotionBehaviors import *
import time
import numpy as np
from math import sin,cos
from underpinnings.BayesianArray import BayesianArray

class CalibrateMotionBehavior(GratbotBehavior):
    def __init__(self):
        motion_list=[]
        ahead_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        for am in ahead_motion:
            motion_list.append(self.wrap_motion_act(MoveAhead(am)))
        turn_motion=[0.2,-0.2,0.3,-0.3,0.4,-0.4]
        for am in turn_motion:
            motion_list.append(self.wrap_motion_act(Turn(am)))
        slew_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        for am in slew_motion:
            motion_list.append(self.wrap_motion_act(Slew(am)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([WaitForStablePose(),Announce("moving"),motion_act,GratbotBehavior_Wait(2.0),WaitForStablePose()])

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)
