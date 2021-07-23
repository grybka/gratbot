from Gyrus import ThreadedGyrus
import numpy as np
import logging
import time
from collections import deque


logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

#if I'm paying attention to a tracked object, follow it with my head
#alternately, hold head level if I'm tilted

class HeadTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        #self.ratio=-0.01473
        self.ratio=20
        self.min_angle_correction=2 #in degrees!
        self.mode="track_first"
        self.allowed_labels=["sports ball","orange"]
        self.max_recent_history=20
        self.servo_angle=deque([ [0,90] ],maxlen=self.max_recent_history)
        self.time_ref=None

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response"]

    def get_name(self):
        return "HeadTrackerGyrus"

    def get_angle_before(self,timestamp):
        for i in range(len(self.servo_angle)-1):
            if self.servo_angle[i][0]<timestamp and self.servo_angle[i+1][0]>timestamp:
                return self.servo_angle[i][1]
        return self.servo_angle[-1][1]

    def read_message(self,message):
        if "packets" in message: #this tracks the
            if self.time_ref==None:
                self.time_ref=-message['timestamp']+message['packets'][-1]['gyroscope_timestamp']
            self.time_ref=max(self.time_ref,-message['timestamp']+message['packets'][-1]['gyroscope_timestamp'])
        if self.time_ref==None:
            return #no reference time
        if "servo_response" in message:
            logger.debug("head angle now {}".format(message["servo_response"]["angle"]))
            self.servo_angle.append([message["timestamp"],message["servo_response"]["angle"]])
        if "tracks" in message:
            if self.tracked_object is None:
                if self.mode=="track_first":
                    #in track first, select whatever the first thing is
                    for track in message["tracks"]:
                        if track["label"] in self.allowed_labels:
                            self.tracked_object=track["id"]
            found_track=None
            for track in message["tracks"]:
                if track["id"]==self.tracked_object:
                    found_track=track
                    break
            if found_track==None: #nothing to look at
                if self.mode=="track_first": #if I'm in track first mode, then forget what I'm tracking
                    self.tracked_object=None
                return

            real_time=message['image_timestamp']-self.time_ref

            center_y=track["center"][1]
            #logger.debug("centery {}".format(center_y))
            error=center_y-0.5
            correction_angle=error*self.ratio
            if abs(correction_angle)>self.min_angle_correction:
                last_angle=self.get_angle_before(real_time)
                logger.debug("head tracker error signal angle {}, should be {}".format(correction_angle,last_angle+correction_angle))
                servo_command={"timestamp": time.time(),"servo_command": {"servo_number":0,"angle": last_angle+correction_angle}}
                self.broker.publish(servo_command,"servo_command")
