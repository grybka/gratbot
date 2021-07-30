from Gyrus import ThreadedGyrus
import numpy as np
import logging
import time
from collections import deque


logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#if I'm paying attention to a tracked object, follow it with my head
#alternately, hold head level if I'm tilted

class MyPID:
    def __init__(self,p,i,d,output_clip=[-1,1]):
        self.const_p=p
        self.const_i=i
        self.const_d=d
        self.history_size=10
        self.history=deque([  ],maxlen=self.history_size)
        self.output_clip=output_clip

    def observe(self,val):
        self.history.append(val)

    def get_response(self):
        if len(self.history)>1:
            return np.clip(self.const_p*self.history[-1]+self.const_d*(self.history[-1]-self.history[-2])+self.const_i*(np.mean(self.history)),self.output_clip[0],self.output_clip[1])
        if len(self.history)==1:
            return np.clip(self.const_p*self.history[-1],self.output_clip[0],self.output_clip[1])
        return 0

def inv_deadzone_func(y,a,b):
    if y==0:
        return 0
    xp=y/b
    if xp>0:
        return xp+a
    return xp-a

class HeadTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        #self.ratio=-0.01473
        #self.pid_controller=MyPID(-17,-3,0,output_clip=[-15,15])
        self.pid_controller=MyPID(-7,0,0,output_clip=[-15,15])
        self.ratio=20
        self.min_angle_correction=1 #in degrees!
        self.mode="track_first" #track_first or off
        self.allowed_labels=["sports ball","orange","face"]
        self.max_recent_history=20
        self.servo_angle=deque([ [0,90] ],maxlen=self.max_recent_history)
        self.time_ref=None
        self.resting_angle=110
        self.time_to_resting=5
        self.last_move=0
        #not used below
        self.rot_vector_history=deque([],maxlen=self.max_recent_history)

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config"]

    def get_name(self):
        return "HeadTrackerGyrus"

    def get_angle_before(self,timestamp):
        for i in range(len(self.servo_angle)-1):
            if self.servo_angle[i][0]<timestamp and self.servo_angle[i+1][0]>timestamp:
                return self.servo_angle[i][1]
        return self.servo_angle[-1][1]

#    def get_rotvec_before(self,timestamp):
#        for i in range(len(self.servo_angle)-1):
#            if self.rot_vector_history[i][0]<timestamp and self.rot_vector_history[i+1][0]>timestamp:
#                return self.rot_vector_history[i][1]
#        return self.rot_vector_history[-1][1]

    def predict_track_pos_change_since(self,start_timestamp):
        slope=-1.55
        delt_rot=np.array(self.rot_vector_history[-1][1])-np.array(self.get_rotvec_before(start_timestamp))
        return delt_rot[1]

    def read_message(self,message):
        if "gyrus_config" in message and message["gyrus_config"]["target_gyrus"]=="HeadTrackerGyrus":
            m=message["gyrus_config"]
            if "mode" in m:
                self.mode=m["mode"]
            if "labels" in m:
                self.allowed_labels=m["labels"]
        if "packets" in message: #this tracks the
            if self.time_ref==None:
                self.time_ref=-message['timestamp']+message['packets'][-1]['gyroscope_timestamp']
            self.time_ref=max(self.time_ref,-message['timestamp']+message['packets'][-1]['gyroscope_timestamp'])
            for packet in message["packets"]:
                self.rot_vector_history.append([packet["gyroscope_timestamp"],packet["local_rotation"]])
        if self.time_ef==None:
            return #no reference time
        if "servo_response" in message:
            logger.debug("head angle now {}".format(message["servo_response"]["angle"]))
            self.servo_angle.append([message["timestamp"],message["servo_response"]["angle"]])
        if "tracks" in message:
            if self.tracked_object is None:
                if self.mode=="track_first":
                    #in track first, select whatever the first thing is
                    for track in message["tracks"]:
                        if track["label"] in self.allowed_labels and track["seen_frames"]>1:
                            self.tracked_object=track["id"]
                    if time.time()-self.last_move>self.time_to_resting:
                        servo_command={"timestamp": time.time(),"servo_command": {"servo_number":0,"angle": 90}}
                        self.broker.publish(servo_command,"servo_command")
                        return

            found_track=None
            for track in message["tracks"]:
                if track["id"]==self.tracked_object:
                    found_track=track
                    break
            if found_track==None: #nothing to look at
                if self.mode=="track_first": #if I'm in track first mode, then forget what I'm tracking
                    self.tracked_object=None
                return

            if self.mode=="off":
                return

            image_time=message['image_timestamp']-self.time_ref
            position_at_image_time=track["center"][1]
            angle_at_image_time=self.get_angle_before(image_time)
            error=position_at_present-0.5

            #position_at_present=position_at_image_time
            #position_at_present=position_at_image_time+self.predict_track_pos_change_since(message['image_timestamp'])
            #TODO should I include velocity here?  maybe maybe not
            #logger.info("position_at_image_time, present {} ,{}".format(position_at_image_time,position_at_present))
            #logger.info("delta pos already = {}".format(position_at_present-position_at_image_time))
            #correction_angle=inv_deadzone_func(error,a,b)
            #logger.debug("centery {}".format(center_y))

            self.pid_controller.observe(error)
            correction_angle=self.pid_controller.get_response()

            if abs(correction_angle)>self.min_angle_correction:
                logger.info("correction angle {}".format(correction_angle))
                new_angle=angle_at_image_time+correction_angle
                #last_angle=self.get_angle_before(real_time)
                #logger.debug("head tracker error signal angle {}, should be {}".format(correction_angle,last_angle+correction_angle))
                servo_command={"timestamp": time.time(),"servo_command": {"servo_number":0,"angle": new_angle}}
                #servo_command={"timestamp": time.time(),"servo_command": {"servo_number":0,"delta_angle": correction_angle}}
                self.broker.publish(servo_command,"servo_command")
                self.last_move=time.time()


class TurnTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        #self.ratio=-0.01473
        self.pid_controller=MyPID(-2,-0,-1)
        self.mode="track_first"
        self.allowed_labels=["sports ball","orange","face"]
        self.max_recent_history=20
        self.servo_angle=deque([ [0,90] ],maxlen=self.max_recent_history)
        self.time_ref=None
        self.min_angle_correction=0.2

    def get_keys(self):
        return ["rotation_vector","tracks","motor_response"]

    def get_name(self):
        return "TurnTrackerGyrus"

    def read_message(self,message):
        if "packets" in message: #this tracks the
            if self.time_ref==None:
                self.time_ref=-message['timestamp']+message['packets'][-1]['gyroscope_timestamp']
            self.time_ref=max(self.time_ref,-message['timestamp']+message['packets'][-1]['gyroscope_timestamp'])
        if self.time_ref==None:
            return #no reference time
        if "motor_response" in message:
            ...
            #self.servo_angle.append([message["timestamp"],message["servo_response"]["angle"]])
        if "tracks" in message:
            if self.tracked_object is None:
                if self.mode=="track_first":
                    #in track first, select whatever the first thing is
                    for track in message["tracks"]:
                        if track["label"] in self.allowed_labels and track["seen_frames"]>1:
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

            center_x=track["center"][0]
            #logger.debug("centery {}".format(center_y))
            error=center_x-0.5
            self.pid_controller.observe(error)
            #correction_angle=error*self.ratio
            correction_angle=self.pid_controller.get_response()
            #logger.info("motor correction: {}".format(correction_angle))
            if abs(correction_angle)>self.min_angle_correction:
                motor_command={"timestamp": time.time(),"motor_command": {"left_throttle":correction_angle,"right_throttle": -correction_angle,"left_duration":0.2,"right_duration": 0.2}}
                self.broker.publish(motor_command,"motor_command")
                self.last_move=time.time()
