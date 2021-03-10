import logging
import time
from datetime import datetime
import math
import torch

def detection_to_activation_zone(x,y,N,M):
    activation=torch.zeros(N*M)
    xi=math.floor(x*N)
    yi=math.floor(y*M)
    return xi+yi*N
    

class GratbotBehaviorNN:
    def __init__(self,comms,eyeballs):
        self.comms=comms
        self.eyeballs=eyeballs
        self.last_eyeball_time=time.time()
        self.after_someone=False
        self.behavior_log=[]

        self.grid_N=5
        self.grid_M=3
        self.last_turns_speed=0.0
        self.last_turns_wheel_yaw=0.5

    def get_state(self,detection_array):
        # For each object I pay attention to
        # An N x M grid
        # last turns speed (self.last_turns_speed)
        # last turns yaw
        state_size=self.grid_N*self.grid_N+2
        state_vector=torch.zeros(state_size)
        #state vector:
        person_activation=torch.zeros(self.grid_N*self.grid_M)
        for detection in detection_array:
            if detection[0]=='person':
                target_x=detection[1]
                target_y=detection[2]
                i=detection_to_activation_zone(x,y,N,M):
                state_vector[i]=1.0
        state_vector[-2]=self.last_turns_speed
        state_vector[-1]=self.last_turns_wheel_yaw

    def act(self):
        #get info from comms and eyeballs
        detection_array,detection_timestamp=self.eyeballs.get_detection_array()
        if detection_timestamp==self.last_eyeball_time:
            return #I've already made a decision on this image, now to wait
        self.last_eyeball_time=detection_timestamp
        state=self.get_state(detection_array)
        #feed state vector into action network I suppose



