
from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
from collections import deque
import numpy as np
import logging
import time
from underpinnings.MotionCorrection import MotionCorrectionRecord

logger=logging.getLogger(__name__)
#logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

#if I'm paying attention to a tracked object, follow it with my head
#alternately, hold head level if I'm tilted
#reworked from headtrackergyrus

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


def get_track_with_id(id,tracks):
    for track in tracks:
        if track["id"]==id:
            return track
    return None

class NeckGazeGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        #Units are degrees per second per pixel
        self.pid_controller=MyPID(-100.,0,0,output_clip=[-100,100])
        self.servo_num=0
        self.motion_corrector=MotionCorrectionRecord()

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse"]

    def get_name(self):
        return "NeckGazeGyrus"

    def get_track_error(self,message):
        track=get_track_with_id(self.tracked_object,message["tracks"])
        if track is None or self.tracked_object is None:
            if len(message["tracks"])!=0:
                self.tracked_object=message["tracks"][0]["id"]
                track=message["tracks"][0]
                logger.debug("new looking at {}".format(id_to_name(track["id"])))
            else:
                logger.debug("Nothing to look at")
                self.tracked_object=None
                return None
        position_at_image=track["center"][1]
        error_signal=180-position_at_image
        ratio=0.19*2*3.14/360 #don't hard code this TODO
        return error_signal*ratio

    def get_pitch_error(self):
        my_pitch=self.motion_corrector.get_pitch()
        return (3.14159/2-my_pitch) #in radians

    def read_message(self,message):
        self.motion_corrector.read_message(message)
#        if "packets" in message: #this tracks the rotation vector
#            if self.time_ref==None:
#                self.time_ref=-message['timestamp']+message['packets'][-1]['gyroscope_timestamp']
#            self.time_ref=max(self.time_ref,-message['timestamp']+message['packets'][-1]['gyroscope_timestamp'])
#            for packet in message["packets"]:
#                self.rot_vector_history.append([packet["gyroscope_timestamp"],packet["local_rotation"]])

        if "clock_pulse" in message:
            if self.tracked_object is None:
                if self.motion_corrector.get_latest_timestamp()==0: #no info yet
                    return
                error_signal=self.get_pitch_error()
                self.pid_controller.observe(error_signal)
                vel=self.pid_controller.get_response()
                logger.debug("Error: {}, vel {}".format(error_signal,vel))
                servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
                self.broker.publish(servo_command,"servo_command")

        if "tracks" in message:
            error_signal=self.get_track_error(message)
            if error_signal is None: #Not following anything
                return
            self.pid_controller.observe(error_signal)
            vel=self.pid_controller.get_response()
            logger.debug("Error: {}, vel {}".format(error_signal,vel))
            servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
            self.broker.publish(servo_command,"servo_command")
