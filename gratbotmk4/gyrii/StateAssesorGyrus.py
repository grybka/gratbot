#collect information and assemble it into an input state vector

#Notes:
#Timescale: n seconds

#What goes in state vector?

#0 head pitch
#1 delta pitch
#2 delta turn
#3 is there a valid track
#4 track y error
#5 track x error




import logging
import time
import math
from Gyrus import ThreadedGyrus
import numpy as np
from underpinnings.MotionCorrection import MotionCorrectionRecord

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

#like motioncorrector but slower
class SAHeadingTracker():
    def __init__(self):
        self.accel=np.array([-10,0,0]) #for gravity z,x,y
        self.z_gyro_index=0
        self.y_gyro_index=2
        self.x_gyro_index=1
        self.last_read_rotation=np.array([0,0,0])
        self.recent_rotation=np.array([0,0,0])

    def read_message(self,message):
        if 'packets' in message: #rotation, etc
            last_packet=message['packets'][-1]
            self.accel=0.7*self.accel+0.3*np.array(last_packet["acceleration"])
            self.recent_rotation=np.array(last_packet["local_rotation"])
            self.rotation_vector=last_packet["rotation_vector"]

    def get_pitch(self):
        return np.arctan2(self.accel[2],-self.accel[0])

    def get_rotation_since_last_check(self):
        heading_a=self.last_read_rotation
        heading_b=self.recent_rotation
        self.last_read_rotation=heading_b
        delta_heading=heading_b-heading_a
        pitch=self.get_pitch()
        cos_angle=np.cos(pitch)
        sin_angle=np.sin(pitch)
        turn_mag=delta_heading[self.z_gyro_index]*cos_angle-delta_heading[self.y_gyro_index]*sin_angle
        return turn_mag,delta_heading[self.x_gyro_index]

class StateAssesorGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.assestment_time_seconds=5
        self.last_assessment_time=time.time()
        ...
        self.heading_tracker=SAHeadingTracker()

    def get_keys(self):
        return ["clock_pulse","rotation_vector","tracks"]

    def get_name(self):
        return "StateAssesorGyrus"

    def update_tracks(self,message):
        tracks=message["tracks"]
        #track_xs=[]
        #track_yz=[]
        #for track in tracks:
        #    track_xs.append(track["center"][0])
        #    track_ys.append(track["center"][1])
        #track_xs=np.digitize(track_xs,[0.33,0.66])
        #track_ys=np.digitize(track_ys,[0.33,0.66])
        #for i in range(len(track_xs)):
        #    ...

    def clock_pulse(self):
        if time.time()>self.last_assessment_time+self.assestment_time_seconds:
            pitch=self.heading_tracker.get_pitch()
            delta_turn,delta_pitch=self.heading_tracker.get_rotation_since_last_check()
            state_vector=[pitch,delta_pitch,delta_turn]

            #logger.info("State Vector {}".format(state_vector))
            message_out={"timestamp": time.time(),"state_vector": state_vector}
            self.broker.publish(message_out,["state_vector"])
            self.last_assessment_time=time.time()
        ...

    def read_message(self,message):
        self.heading_tracker.read_message(message)
        if "clock_pulse" in message:
            self.clock_pulse()
        if "tracks" in message:
            self.update_tracks(message)
