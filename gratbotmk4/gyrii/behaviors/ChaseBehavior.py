
from gyrii.behaviors.Behavior import *
import time
import numpy as np
from math import sin,cos
import random
import logging
from underpinnings.id_to_name import id_to_name

logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
logger.setLevel(logging.INFO)

class ChaseObject(GratbotBehavior):
    def __init__(self,id):
        self.to_track=id
        self.next_timestamp=0
        self.target_distance=1.0
        self.track_angle_to_real_angle=1.515

    def get_motion(self,angle_error,angle_velocity,dist_error,dist_velocity,dur):
        angle_tolerance=0.1
        if abs(angle_error)<angle_tolerance:
            angle_error=0
        target_angle_velocity=angle_velocity+angle_error/dur #catch up in
        #target_angle_velocity=angle_error/dur #catch up in third a second
        #z_gyro_per_motor=1.83
        z_gyro_per_motor=2*1.83  #remember there are two motors!
        angle_left_excitation=target_angle_velocity/z_gyro_per_motor

        #dist_tolerance=0.1*self.target_distance #I have more control than this, but I don't have the depth perception
        #if abs(dist_error)<dist_tolerance:
        #    dist_error=0
        #unlike angle error, we can't sensibly go more than about 40 cm in a step, so clip dist dist
        dist_error=np.clip(dist_error,-0.4,0.4)

        target_fb_velocity=dist_velocity+dist_error/dur
        fb_per_motor=1.2
        fb_left_excitation=target_fb_velocity
        return angle_left_excitation+fb_left_excitation,-angle_left_excitation+fb_left_excitation

    def act(self,**kwargs):
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            if kwargs["short_term_memory"]["tracks"]["timestamp"]<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info
            self.last_tracks_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            for track in tracks:
                if track["id"]==self.to_track:
                    angle_error=(track["center"][0]-0.5)*self.track_angle_to_real_angle  #in radians/fov where fove is like 1.2
                    angle_velocity=track["velocity"][0]*self.track_angle_to_real_angle
                    dist_error=track["center"][2]-self.target_distance
                    dist_velocity=track["velocity"][2]

                    #this sholud be its own behavior but for funsies
#                    if abs(angle_error)<0.1 and abs(dist_error)<0.1*self.target_distance:
#                        self.target_distance=0
#                        dist_error=track["center"][2]-self.target_distance

                    dur=0.3
                    left,right=self.get_motion(angle_error,angle_velocity,dist_error,dist_velocity,dur)
                    if min(abs(left),abs(right))<0.4:
                        logger.debug("motion too slow, trying faster")
                        dur=0.1
                        left,right=self.get_motion(angle_error,angle_velocity,dist_error,dist_velocity,dur)
                    maxval=max(abs(left),abs(right))
                    logger.debug("chase motor raw {},{}".format(left,right))
                    if maxval>1.0:
                        left=left/maxval
                        right=right/maxval
                    logger.debug("chase motor scaled {},{}".format(left,right))
                    broker=kwargs["broker"]
                    motor_command={"left_throttle": left, "right_throttle": right, "left_duration": dur, "right_duration": dur}
                    if max(abs(left),abs(right))>0.2:
                        broker.publish({"timestamp": time.time(),"motor_command": motor_command},"motor_command")

                    #if I'm not turning too often, I can count on tracking while moving
                    if abs(left-right)<0.4:
                        self.next_timestamp=time.time()+dur*0.75
                    else:
                        self.next_timestamp=time.time()+dur
                    return GratbotBehaviorStatus.INPROGRESS


class TrackObject(GratbotBehavior):
    def __init__(self,id):
        self.to_track=id
        self.next_timestamp=0
        self.mode="turn_only"
        #self.last_tracks_timestamp=0

    def turn_speed_to_catch(self,turn_speed_to_match,error,catch_up_time):
        turn_speed_to_request=turn_speed_to_match+error/catch_up_time #catch up in third a second
        angle_heading_slope=1.515
        desired_z_gyro=turn_speed_to_request/angle_heading_slope
        z_gyro_per_motor=1.83
        desired_motor=desired_z_gyro/z_gyro_per_motor
        return desired_motor

    def act(self,**kwargs):
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]

            if kwargs["short_term_memory"]["tracks"]["timestamp"]<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info

            self.last_tracks_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            for track in tracks:
                if track["id"]==self.to_track:
                    if self.mode=="turn_only":
                        error=(track["center"][0]-0.5)  #in radians/fov where fove is like 1.2
                        dur=0.3
                        desired_motor=self.turn_speed_to_catch(track["velocity"][0],error*0.95,dur)
                        if abs(desired_motor)<0.3:
                            dur=0.1
                            desired_motor=self.turn_speed_to_catch(track["velocity"][0],error*0.95,dur)

                        logger.debug("following after missed frames {}".format(track["missed_frames"]))
                        logger.debug("object found with error {}".format(error))
                        logger.debug("motor {} dur {}".format(desired_motor,dur))
                        broker=kwargs["broker"]
                        motor_command={"left_throttle": desired_motor, "right_throttle": -desired_motor, "left_duration": dur*0.9, "right_duration": dur*0.9}
                        broker.publish({"timestamp": time.time(),"motor_command": motor_command},"motor_command")
                        self.next_timestamp=time.time()+dur*0.9
                        return GratbotBehaviorStatus.INPROGRESS

        logger.debug("Tracker lost {}".format(id_to_name(self.to_track)))
        return GratbotBehaviorStatus.FAILED

class TrackIfSeen(GratbotBehavior):
    def __init__(self):
        self.to_track=["sports ball","orange"]
        #self.to_track=["person"]
        self.sub_behavior=None

    def act(self,**kwargs):
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.INPROGRESS:
                return step_status
            if step_status==GratbotBehaviorStatus.FAILED:
                self.sub_behavior=None
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            for track in tracks:
                if track["label"] in self.to_track:
                    logger.debug("Found a {} with ID {} to track".format(track["label"],id_to_name(track["id"])))
                    #self.sub_behavior=TrackObject(track["id"])
                    self.sub_behavior=ChaseObject(track["id"])
                    break
        return GratbotBehaviorStatus.INPROGRESS
