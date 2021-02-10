from behaviors.GratbotBehavior import GratbotBehavior
from behaviors.GratbotBehavior import GratbotBehavior_Loop
from behaviors.GratbotBehavior import GratbotBehavior_Wait
from behaviors.GratbotBehavior import GratbotBehaviorStatus
from behaviors.GratbotBehavior import GratbotBehavior_Series
from behaviors.GratbotBehavior import GratbotBehavior_RecordSensorToMemory
from behaviors.GratbotBehavior import GratbotBehavior_CopyMemory
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random
import json
import sys,os,traceback

import logging
import time


class TurnFixedAmount(GratbotBehavior):
    def __init__(self,angle):
        super().__init__()
        self.angle=angle
        self.reset()

    def act(self,comms,sensors):
        if time.time()<self.retry_in:
            return GratbotBehaviorStatus.INPROGRESS
        max_turn_angle=sensors.turn_predictor.get_max_turn_angle()
        if abs(max_turn_angle)>abs(self.remaining_angle):
            sensors.send_command_turn_angle(comms,self.remaining_angle)
            self.reset()
            return GratbotBehaviorStatus.COMPLETED
        else:
            toturn=np.sign(self.remaining_angle)*max_turn_angle
            self.remaining_angle-=toturn
            sensors.send_command_turn_angle(comms,toturn)
            self.retry_in=time.time()+0.1 #HARDCODED wait for motor to act
            return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.remaining_angle=self.angle
        self.retry_in=time.time()

class TurnToHeading(GratbotBehavior):
    def __init__(self,heading,target_uncertainty):
        super().__init__()
        self.target_heading=self.wrap_angle(heading)
        self.target_uncertainty=target_uncertainty
        self.motor_wait_time=0.1
        self.compass_wait_time=0.1
        self.timeout=5.0
        self.reset()

    def wrap_angle(self,x):
        while x>np.pi:
            x-=2*np.pi
        while x<-np.pi:
            x+=2*np.pi
        return x

    def min_angle_difference(self,x,y):
        x=self.wrap_angle(x)
        y=self.wrap_angle(y)
        a=self.wrap_angle(x-y)
        b=self.wrap_angle(x-y+2*np.pi)
        c=self.wrap_angle(x-y-2*np.pi)
        r=[a,b,c]
        i=np.argmin(np.abs(r))
        return r[i]


    def act(self,comms,sensors):
        if self.start_time==None:
            self.start_time=time.time()
        if time.time()-self.start_time>self.timeout:
            print("turn to heading failed")
            return GratbotBehaviorStatus.FAILED
        if time.time()<self.wait_until:
            return GratbotBehaviorStatus.INPROGRESS
        heading,heading_unc=sensors.map.get_heading()
        #print("Planning turn to {}, measured {} +- {}".format(self.target_heading,heading,heading_unc))
        #if I am within my desired uncertainty of heading, and that uncertainty is less than target, I'm done
        if abs(self.min_angle_difference(self.target_heading,heading))<self.target_uncertainty and heading_unc<self.target_uncertainty:
            #print("I'm there")
            return GratbotBehaviorStatus.COMPLETED
        #if I don't know my heading well enough, wait until it settles
        if heading_unc>self.target_uncertainty:
            #print("Uncertainty too high, waiting")
            self.wait_until=time.time()+self.compass_wait_time
            return GratbotBehaviorStatus.INPROGRESS
        #I know my heading well but it isn't where I want
        delta_angle=self.min_angle_difference(self.target_heading,heading)
        #print("turning {}".format(delta_angle))
        sensors.send_command_turn_angle(comms,delta_angle)
        self.wait_until=time.time()+self.motor_wait_time
        return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.start_time=None
        self.wait_until=time.time() #use if I'm waiting for motors or next compass heading

class ForwardFixedAmount(GratbotBehavior):
    def __init__(self,dist): #in meters
        super().__init__()
        self.dist=dist
        self.motor_wait_time=0.1
        self.reset()

    def act(self,comms,sensors):
        if time.time()<self.retry_in:
            return GratbotBehaviorStatus.INPROGRESS
        if self.dist_remaining<=0:
            self.reset()
            return GratbotBehaviorStatus.COMPLETED
        max_dist=sensors.fb_predictor.get_max_fb_dist()
        #print("distance remaining {}".format(self.dist_remaining))
        if abs(max_dist)>abs(self.dist_remaining):
            sensors.send_command_forward_meters(comms,np.sign(self.dist)*self.dist_remaining)
            self.dist_remaining=0
        else:
            tomove=np.sign(self.dist_remaining)*abs(max_dist)
            self.dist_remaining-=abs(tomove)
            sensors.send_command_forward_meters(comms,tomove)
        self.retry_in=time.time()+self.motor_wait_time
        return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.dist_remaining=abs(self.dist)
        self.retry_in=time.time()
