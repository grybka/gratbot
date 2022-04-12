
from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
from collections import deque
import numpy as np
import logging
import time
from underpinnings.MotionCorrection import MotionCorrectionRecord

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)
#logger.setLevel(logging.DEBUG)


def get_track_with_id(id,tracks):
    for track in tracks:
        if track["id"]==id:
            return track
    return None

#Takes info from tracks and IMU
#Outputs an error signal between my current y heading and desired y heading
#Currently either picks an object to track, or keeps head level
#I should separate out this from the thing that decides what to look at
class PointingErrorGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        self.motion_corrector=MotionCorrectionRecord()
        self.last_track_time=0
        self.track_time_declare_lost=2
        self.resting_angle=30*(2*3.14/360)
        self.state="WAITING" # or LOOKING_AT_SOMETHING or LEVELLING
        self.wait_time=2
        self.wait_start_time=time.time()

        self.do_distance_corrections=False
        self.target_distance=1.0
        #for debug messages
        self.last_report_time=time.time()
        self.report_period=1

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse"]

    def get_name(self):
        return "PointingErrorGyrus"

    def get_track_error(self,message):
        track=get_track_with_id(self.tracked_object,message["tracks"])
        if track is None:
            return None,None
        if track["info"]=="LOST":
            logger.info("Track lost")
            self.tracked_object=None
            return None,None,None
        else:
            logger.debug("track status {}".format(track["info"]))
        yposition_at_image=track["center"][1]
        xposition_at_image=track["center"][0]
        yerror_signal=yposition_at_image-0.5
        xerror_signal=xposition_at_image-0.5
        ratio=2*2*3.14*60/360 #don't hard code this TODO
        disterror=0
        if "spatial_array" in track:
            zdist=track["spatial_array"][2]
            disterror=zdist-self.target_distance
        return -xerror_signal*ratio,-yerror_signal*ratio,disterror

    def get_pitch_error(self):
        my_pitch=self.motion_corrector.get_pitch()
        #logger.debug("pitch {}".format(my_pitch))
        return -my_pitch+self.resting_angle #in radians

    def report_error(self,xerror,yerror,disterror=0):
        #xerror and yerror should be roughly in degrees
        #logger.debug("Reporting error {} {}".format(xerror,yerror))
        message_out={"timestamp": time.time(),"pointing_error_x": xerror,"pointing_error_y": yerror, "distance_error": disterror}
        self.broker.publish(message_out,["pointing_error_x","pointing_error_y","distance_error"])

    def find_new_object(self,message):
        tracks=message["tracks"]
        if len(tracks)==0:
            return None
        for i in range(len(tracks)):
            closest_point=10000
            best_track={"id":None}
            if tracks[i]["info"]=="DETECTED" or tracks[i]["info"]=="EXITED":
                dx=tracks[i]["center"][0]-0.5
                dy=tracks[i]["center"][1]-0.5
                dist=dx*dx+dy*dy
                if dist<closest_point:
                    best_track=tracks[i]
                    closest_point=dist
        if best_track["id"]!=None:
            self.state="LOOKING_AT_SOMETHING"
            self.tracked_object=best_track["id"]
            logger.info("new looking at {}: {}".format(best_track["label"],best_track["info"]))
        return best_track["id"]

    def read_message(self,message):
        self.motion_corrector.read_message(message)
        if "clock_pulse" in message:
            if self.last_report_time+self.report_period<time.time():
                logger.info("Pointing state {}".format(self.state))
                self.last_report_time=time.time()

            #if I'm no tracking something, hold head level
            if self.state=="LEVELLING":
                if self.motion_corrector.get_latest_timestamp()==0: #no info yet
                    return
                error_signal=self.get_pitch_error()
                self.report_error(0,error_signal)
            elif self.state=="WAITING":
                if time.time()-self.wait_start_time>self.wait_time:
                    self.state="LEVELLING"
            else:
                #if I haven't heard from the thing I'm tracking in a long time, drop it
                if time.time()-self.last_track_time>self.track_time_declare_lost:
                    logger.info("Track timed out without being reported as lost {}".format(time.time()-self.last_track_time))
                    self.tracked_object=None
                    self.state="WAITING"
                    self.wait_start_time=time.time()

        if "tracks" in message:
            if self.state=="LEVELLING":
                self.find_new_object(message)
            if self.state=="LOOKING_AT_SOMETHING":
                xerror_signal,yerror_signal,disterror_signal=self.get_track_error(message)
                if yerror_signal is None: #Not following anything or wasn't in this message
                    if self.tracked_object==None:
                        logger.debug("Ihavenotracked object")
                    #logger.debug("not following anything")
                        self.state="WAITING"
                        self.wait_start_time=time.time()
                    return
                self.last_track_time=time.time()
                self.report_error(xerror_signal,yerror_signal)


class MyPID:
    def __init__(self,p,i,d,output_clip=[-1,1]):
        self.const_p=p
        self.const_i=i
        self.const_d=d
        self.history_size=10
        self.history=deque([  ],maxlen=self.history_size)
        #set zero error as the history
        for i in range(self.history_size):
            self.history.append(0)
        self.output_clip=output_clip

    def observe(self,val):
        self.history.append(val)

    def get_response(self):
        if len(self.history)>1:
            return np.clip(self.const_p*self.history[-1]+self.const_d*(self.history[-1]-self.history[-2])+self.const_i*(np.mean(self.history)),self.output_clip[0],self.output_clip[1])
        if len(self.history)==1:
            return np.clip(self.const_p*self.history[-1],self.output_clip[0],self.output_clip[1])
        return 0

#Take in reports of pointing error and correct them
class NeckPointingErrorCorrectionGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        #self.tracked_object=None
        #Units are degrees per second per pixel
        #self.pid_controller=MyPID(-200.,0,0,output_clip=[-150,150])
        self.pid_controller=MyPID(-100.,-10,-400,output_clip=[-150,150])
        self.servo_num=0
        #self.motion_corrector=MotionCorrectionRecord()
        #self.last_track_time=0

    def get_keys(self):
        return ["pointing_error_y"]

    def get_name(self):
        return "NeckPointingErrorCorrectionGyrus"

    def read_message(self,message):
        if "pointing_error_y" in message:
            error_signal=message["pointing_error_y"]
            self.pid_controller.observe(error_signal)
            vel=self.pid_controller.get_response()
            servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
            self.broker.publish(servo_command,"servo_command")

class BodyPointingErrorCorrectionGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        #Units are throttle per radian
        self.turn_pid_controller=MyPID(-0.20,-0.15,-0.2,output_clip=[-1,1])
        #units are throttle per meter
        self.distance_pid_controller=MyPID(-0.5,0,0,output_clip=[-1,1])

    def get_keys(self):
        return ["pointing_error_x"]

    def get_name(self):
        return "BodyPointingErrorCorrectionGyrus"

    def read_message(self,message):
        if "pointing_error_x" in message:
            error_signal=message["pointing_error_x"]
            #logger.debug("Body Pointing Error {}".format(error_signal))
            self.turn_pid_controller.observe(error_signal)
            turn_vel=self.turn_pid_controller.get_response()
            dist_vel=0

            if "distance_error" in message:
                self.distance_pid_controller.observe(message["distance_error"])
                dist_vel=self.distance_pid_controller.get_response()

            left_throttle=-turn_vel+dist_vel
            right_throttle=turn_vel+dist_vel
            #logger.info("sending x velocity {}".format(vel))
            #dur=0.2 In the current configuration, this should be an honest assestment of when I will make the next
            #correction
            #dur=0.05 #at 20 fps
            dur=0.06 #there is lag intalking to the motor
            motor_command={"timestamp": time.time(),"motor_command": {"left_throttle":left_throttle,"right_throttle": right_throttle,"left_duration":dur,"right_duration": dur}}
            #logger.info("publishing motor command")
            self.broker.publish(motor_command,"motor_command")
