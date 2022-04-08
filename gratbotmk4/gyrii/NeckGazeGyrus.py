
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
class PointingErrorGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_object=None
        self.motion_corrector=MotionCorrectionRecord()
        self.last_track_time=0
        self.resting_angle=30*(2*3.14/360)

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse"]

    def get_name(self):
        return "PointingErrorGyrus"

    def get_track_error(self,message):
        track=get_track_with_id(self.tracked_object,message["tracks"])
        if track is None or self.tracked_object is None:
            if len(message["tracks"])!=0:
                self.tracked_object=message["tracks"][0]["id"]
                track=message["tracks"][0]
                #logger.debug("new looking at {}".format(id_to_name(track["id"])))
                logger.info("new looking at {}".format(track["label"]))
            else:
                logger.debug("Nothing to look at")
                self.tracked_object=None
                return None,None
        if track["info"]=="LOST":
            logger.debug("Track lost")
            self.tracked_object=None
            return None,None
        else:
            logger.debug("track status {}".format(track["info"]))
        yposition_at_image=track["center"][1]
        xposition_at_image=track["center"][0]
        yerror_signal=yposition_at_image-0.5
        xerror_signal=xposition_at_image-0.5
        ratio=2*2*3.14*60/360 #don't hard code this TODO
        return -xerror_signal*ratio,-yerror_signal*ratio

    def get_pitch_error(self):
        my_pitch=self.motion_corrector.get_pitch()
        #logger.debug("pitch {}".format(my_pitch))
        return -my_pitch+self.resting_angle #in radians

    def report_error(self,xerror,yerror):
        #xerror and yerror should be roughly in degrees
        logger.debug("Reporting error {} {}".format(xerror,yerror))
        message_out={"timestamp": time.time(),"pointing_error_x": xerror,"pointing_error_y": yerror}
        self.broker.publish(message_out,["pointing_error_x","pointing_error_y"])

    def read_message(self,message):
        self.motion_corrector.read_message(message)
        if "clock_pulse" in message:
            #if I'm no tracking something, hold head level
            if self.tracked_object is None or time.time()>(self.last_track_time+1):
                self.tracked_object=None
                if self.motion_corrector.get_latest_timestamp()==0: #no info yet
                    return
                error_signal=self.get_pitch_error()
                self.report_error(0,error_signal)

        if "tracks" in message:
            self.last_track_time=time.time()
            xerror_signal,yerror_signal=self.get_track_error(message)
            if yerror_signal is None: #Not following anything
                logger.debug("not following anything")
                return
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
        #self.pid_controller=MyPID(-0.10,-0.3,0,output_clip=[-1,1])
        #self.pid_controller=MyPID(-0.20,0,-0.15,output_clip=[-1,1])
        self.pid_controller=MyPID(-0.10,0,0,output_clip=[-1,1])

    def get_keys(self):
        return ["pointing_error_x"]

    def get_name(self):
        return "BodyPointingErrorCorrectionGyrus"


    def read_message(self,message):
        if "pointing_error_x" in message:
            error_signal=message["pointing_error_x"]
            logger.debug("Body Pointing Error {}".format(error_signal))
            self.pid_controller.observe(error_signal)
            vel=self.pid_controller.get_response()

            left_throttle=-vel
            right_throttle=vel
            logger.debug("x velocity {}".format(vel))
            dur=0.2
            motor_command={"timestamp": time.time(),"motor_command": {"left_throttle":left_throttle,"right_throttle": right_throttle,"left_duration":dur,"right_duration": dur}}
            #logger.info("publishing motor command")
            self.broker.publish(motor_command,"motor_command")

#class NeckGazeGyrus(ThreadedGyrus):
#    def __init__(self,broker):
#        super().__init__(broker)
#        self.tracked_object=None
#        #Units are degrees per second per pixel
#        #self.pid_controller=MyPID(-200.,0,0,output_clip=[-150,150])
#        self.pid_controller=MyPID(-100.,-10,-400,output_clip=[-150,150])
#        self.servo_num=0
#        self.motion_corrector=MotionCorrectionRecord()
#        self.last_track_time=0
#
#    def get_keys(self):
#        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse"]
#
#    def get_name(self):
#        return "NeckGazeGyrus"
#
#    def get_track_error(self,message):
#        track=get_track_with_id(self.tracked_object,message["tracks"])
#        if track is None or self.tracked_object is None:
#            if len(message["tracks"])!=0:
#                self.tracked_object=message["tracks"][0]["id"]
#                track=message["tracks"][0]
#                logger.debug("new looking at {}".format(id_to_name(track["id"])))
#            else:
#                logger.debug("Nothing to look at")
#                self.tracked_object=None
#                return None
#        if track["info"]=="LOST":
#            logger.debug("Track lost")
#            self.tracked_object=None
#            return None
#        else:
#            logger.debug("track status {}".format(track["info"]))
#        position_at_image=track["center"][1]
#        error_signal=position_at_image-0.5
#        ratio=2*2*3.14*60/360 #don't hard code this TODO
#        return -error_signal*ratio
#
#    def get_pitch_error(self):
#        my_pitch=self.motion_corrector.get_pitch()
#        #logger.debug("pitch {}".format(my_pitch))
#        return -my_pitch #in radians
#
#    def read_message(self,message):
#        self.motion_corrector.read_message(message)
#
#        if "clock_pulse" in message:
#            if self.tracked_object is None or time.time()>(self.last_track_time+1):
#                self.tracked_object=None
#                if self.motion_corrector.get_latest_timestamp()==0: #no info yet
#                    return
#                error_signal=self.get_pitch_error()
#                self.pid_controller.observe(error_signal)
#                vel=self.pid_controller.get_response()
#                #logger.debug("Error: {}, vel {}".format(error_signal,vel))
#                servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
#                self.broker.publish(servo_command,"servo_command")
#            #else:
#            #    logger.debug("tracked object is {}".format(id_to_name(self.tracked_object)))
#
#        if "tracks" in message:
#            self.last_track_time=time.time()
#            error_signal=self.get_track_error(message)
#            if error_signal is None: #Not following anything
#                logger.debug("not following anything")
#                return
#            self.pid_controller.observe(error_signal)
#            vel=self.pid_controller.get_response()
#            logger.debug("Track Error: {}, vel {}".format(error_signal,vel))
#            servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
#            self.broker.publish(servo_command,"servo_command")
