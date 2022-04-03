
from Gyrus import ThreadedGyrus
from underpinnings import id_to_name
import numpy as np
import logging
import time

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
        self.pid_controller=MyPID(1,0,0,output_clip=[-10,10])
        self.servo_num=0

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse"]

    def get_name(self):
        return "NeckGazeGyrus"

    def read_message(self,message):
        if "packets" in message: #this tracks the rotation vector
            if self.time_ref==None:
                self.time_ref=-message['timestamp']+message['packets'][-1]['gyroscope_timestamp']
            self.time_ref=max(self.time_ref,-message['timestamp']+message['packets'][-1]['gyroscope_timestamp'])
            for packet in message["packets"]:
                self.rot_vector_history.append([packet["gyroscope_timestamp"],packet["local_rotation"]])

        if "tracks" in message:
            #If I've lost my last track, find something new
            track=get_track_with_id(self.tracked_object,message["tracks"])
            if track is None or self.tracked_object is None:
                if len(message["tracks"])!=0:
                    self.tracked_object=message["tracks"][0]["id"]
                    track=message["tracks"][0]
                    logger.debug("new looking at {}".format(id_to_name(track["id"])))
                else:
                    logger.debug("Nothing to look at")
                    return #TODO I should switch to level here
            position_at_image_time=track["center"][1]
            error_signal=180-position_at_image
            self.pid_controller.observe(error)
            vel=self.pid_controller.get_response()
            servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
            broker.publish(motor_command,"servo_command")
