from gyrii.behaviors.Behavior import *
from gyrii.behaviors.FocusBehavior import *
import time
import numpy as np
from math import sin,cos
import random
import logging
from underpinnings.id_to_name import id_to_name

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class ServoVelUpAndDown(GratbotBehavior):
    def __init__(self):
        self.vel_amplitude=100
        self.period=10
        self.servo_num=0

    def act(self,**kwargs):
        broker=kwargs["broker"]
        t=time.time()
        vel=self.vel_amplitude*np.sin(2*np.pi*t/self.period)
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"vel": vel}}
        broker.publish(servo_command,"servo_command")
        return GratbotBehaviorStatus.INPROGRESS,{}

class RunMotors(GratbotBehavior):
    def __init__(self,lmotor,rmotor,duration):
        self.lmotor=lmotor
        self.rmotor=rmotor
        self.duration=duration

    def act(self,**kwargs):
        broker=kwargs["broker"]
        motor_command={"timestamp": time.time(),"motor_command": {"left_throttle": self.lmotor,
                                                                   "right_throttle": self.rmotor,
                                                                   "left_duration": self.duration,
                                                                   "right_duration": self.duration},"keys": ["motor_command"]}
        #logging.info("Emitting Motor Command {}".format(motor_command))
        broker.publish(motor_command,"motor_command")
        return GratbotBehaviorStatus.COMPLETED,{}

class RunServo(GratbotBehavior):
    def __init__(self,servo_num,servo_angle):
        self.servo_num=servo_num
        self.servo_angle=servo_angle
    def act(self,**kwargs):
        broker=kwargs["broker"]
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"angle": self.servo_angle}}
        broker.publish(servo_command,"servo_command")
        return GratbotBehaviorStatus.COMPLETED,{}

class RunServoDelta(GratbotBehavior):
    def __init__(self,servo_num,delta_servo_angle):
        self.servo_num=servo_num
        self.delta_servo_angle=delta_servo_angle
    def act(self,**kwargs):
        broker=kwargs["broker"]
        #logger.debug("servo delta {}".format(self.delta_servo_angle))
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"delta_angle": self.delta_servo_angle}}
        broker.publish(servo_command,"servo_command")
        return GratbotBehaviorStatus.COMPLETED,{}

def extract_track_with_label(short_term_memory,labels):
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["label"] in labels:
                return track
    return None

def extract_track_with_id(short_term_memory,id):
    if "tracks" in short_term_memory:
        for track in short_term_memory["tracks"]["tracks"]:
            if track["id"]==id:
                return track
    return None

class FocusOnObjectOfLabel(GratbotBehavior):
    def __init__(self,labels):
        self.labels=labels
    def act(self,**kwargs):
        #logger.debug("FocusOnObjectOfLabel start")
        if "focus_track_id" in kwargs["state"]:
            to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
            if to_track["label"] in self.labels:
                return GratbotBehaviorStatus.COMPLETED,{"focus_track_id": to_track["id"]}
            else:
                kwargs["state"]["focus_track_id"]=None
        to_track=extract_track_with_label(kwargs["short_term_memory"],self.labels)
        if to_track is None:
            return GratbotBehaviorStatus.FAILED,{"focus_track_id": None}
        kwargs["state"]["focus_track"]=to_track["id"]
        return GratbotBehaviorStatus.COMPLETED,{"focus_track_id": to_track["id"]}


class ServoUpAndDown(GratbotBehavior):
    def __init__(self,servo_num=0,servo_step=2,servo_max_angle=160,servo_min_angle=65,wait_time=0.25):
        self.servo_num=servo_num
        self.servo_step=servo_step
        self.servo_max_angle=servo_max_angle
        self.servo_min_angle=servo_min_angle
        self.up_not_down=True
        self.next_act_time=0
        self.wait_time=wait_time
    def act(self,**kwargs):
        if time.time()<self.next_act_time:
            logger.debug("up and down waiting")
            return GratbotBehaviorStatus.INPROGRESS, {}
        #reverse direction if needed
        to_do=None
        if "servo_angle" in kwargs["state"]:
            if self.up_not_down:
                if kwargs["state"]["servo_angle"][self.servo_num]>self.servo_max_angle:
                    logger.debug("up and down change to down")
                    self.up_not_down=False
            else:
                if kwargs["state"]["servo_angle"][self.servo_num]<self.servo_min_angle:
                    self.up_not_down=True
                    logger.debug("up and down change to up")
        else:
            logger.debug("up and down centering")
            midpoint=int(0.5*(self.servo_max_angle+self.servo_min_angle))
            to_do=RunServo(self.servo_num,midpoint)
        #instruct servos to move
        logger.debug("up and down moving")
        if to_do is None:
            to_do=RunServoDelta(self.servo_num,self.servo_step if self.up_not_down else -self.servo_step)
        if to_do.act(**kwargs) == GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED, {}
        self.next_act_time=time.time()+self.wait_time
        return GratbotBehaviorStatus.INPROGRESS, {}

class NeckCenterFocusObject(GratbotBehavior):
    def __init__(self,servo_step=2,servo_max_angle=160,servo_min_angle=65,wait_time=0.20,target_position=0.5):
        self.servo_num=0
        self.servo_step=servo_step
        self.servo_max_angle=servo_max_angle
        self.servo_min_angle=servo_min_angle
        self.wait_time=wait_time
        self.next_act_time=0
        self.close_enough=0.1
        self.target_position=target_position
    def act(self,**kwargs):
        if time.time()<self.next_act_time:
            return GratbotBehaviorStatus.INPROGRESS,{}
        #logger.debug("NeckCenterFocusObject start")
        if "focus_track_id" not in kwargs:
            logger.warning("focus track id not in kwargs")
            return GratbotBehaviorStatus.FAILED, {}
        if kwargs["focus_track_id"]==None:
            logger.warning("No track to focus neck on")
            return GratbotBehaviorStatus.FAILED, {}
        #logger.debug("focus track id {}".format(kwargs["focus_track_id"]))
        to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
        if to_track==None:
            logger.debug("Neckcenterfocusobject no lost track")
            return GratbotBehaviorStatus.FAILED, {}
        center_y=to_track["center"][1]
        error=center_y-self.target_position
        if abs(error)<self.close_enough:
            logger.debug("Neckcenterfocusobject completed with error {}".format(error))
            return GratbotBehaviorStatus.COMPLETED, {}
        #logger.debug("center is {} servo step is {}".format(to_trock["center"][1],-self.servo_step if error<0 else self.servo_step))
        to_do=RunServoDelta(self.servo_num,-self.servo_step if error<0 else self.servo_step)
        if to_do.act(**kwargs) == GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED, {}
        self.next_act_time=time.time()+self.wait_time
        return GratbotBehaviorStatus.INPROGRESS, {}

class TurnCenterFocusObject(GratbotBehavior):
    def __init__(self,wait_time=0.20,target_position=0.5):
        self.wait_time=wait_time
        self.next_act_time=0
        self.close_enough=0.1
        self.target_position=target_position

    def act(self,**kwargs):
        if time.time()<self.next_act_time:
            return GratbotBehaviorStatus.INPROGRESS,{}
        logger.debug("TurnCenterFocusObject start")
        if "focus_track_id" not in kwargs:
            logger.warning("focus track id not in kwargs")
            return GratbotBehaviorStatus.FAILED, {}
        if kwargs["focus_track_id"]==None:
            logger.warning("No track to focus neck on")
            return GratbotBehaviorStatus.FAILED, {}
        #logger.debug("focus track id {}".format(kwargs["focus_track_id"]))
        to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
        if to_track==None:
            logger.debug("Turn enterfocusobject no lost track")
            return GratbotBehaviorStatus.FAILED, {}
        center_y=to_track["center"][0]
        error=center_y-self.target_position
        if abs(error)<self.close_enough:
            logger.debug("Turn enterfocusobject completed with error {}".format(error))
            return GratbotBehaviorStatus.COMPLETED, {}
        m=0.4 if error<0 else -0.4
        logger.debug("Run motors {}".format(m))
        to_do=RunMotors(m,-m,0.1)
        if to_do.act(**kwargs) == GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED, {}
        self.next_act_time=time.time()+self.wait_time
        return GratbotBehaviorStatus.INPROGRESS, {}

class NoteInitialTrackPos(GratbotBehavior):
    def __init__(self,xcoord,init_loc="neck_calib_init"):
        self.init_loc=init_loc
        self.xcoord=xcoord

    def act(self,**kwargs):
        if "focus_track_id" not in kwargs:
            logger.warning("focus track id not in kwargs")
            return GratbotBehaviorStatus.FAILED, {}
        if kwargs["focus_track_id"]==None:
            logger.warning("No track to focus neck on")
            return GratbotBehaviorStatus.FAILED, {}
        to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
        init_vec=[time.time(),to_track["center"],kwargs["short_term_memory"]["packets"]["packets"][0],self.xcoord]
        kwargs["short_term_memory"][self.init_loc]=init_vec
        return GratbotBehaviorStatus.COMPLETED, {}

class RecordFinalTrackPos(GratbotBehavior):
    def __init__(self,xcoord,init_loc="neck_calib_init",mem_loc="neck_calib"):
        self.init_loc=init_loc
        self.final_loc=mem_loc
        self.xcoord=xcoord
    def act(self,**kwargs):
        if "focus_track_id" not in kwargs:
            logger.warning("focus track id not in kwargs")
            return GratbotBehaviorStatus.FAILED, {}
        if kwargs["focus_track_id"]==None:
            logger.warning("No track to focus neck on")
            return GratbotBehaviorStatus.FAILED, {}
        if self.init_loc not in kwargs["short_term_memory"]:
            logger.warning("Trying to record final state with no initial state")
            return GratbotBehaviorStatus.FAILED, {}
        #if self.last_xcoord_loc not in kwargs["short_term_memory"]:
        #    logger.warning("Trying to record final state with no initial state")
        #    return GratbotBehaviorStatus.FAILED, {}
        to_track=extract_track_with_id(kwargs["short_term_memory"],kwargs["focus_track_id"])
        if self.final_loc not in kwargs["short_term_memory"]:
             kwargs["short_term_memory"][self.final_loc]=[]
        final_vec=[time.time(),to_track["center"],kwargs["short_term_memory"]["packets"]["packets"][0],self.xcoord]
        kwargs["short_term_memory"][self.final_loc].append([kwargs["short_term_memory"][self.init_loc],final_vec])
        return GratbotBehaviorStatus.COMPLETED, {}

class BroadcastCalibration(GratbotBehavior):
    def __init__(self,data_loc):
        self.data_loc=data_loc
    def act(self,**kwargs):
        broker=kwargs["broker"]
        calib_note={"timestamp": time.time(), "logged_note":
                        {"name": "neck servo calibration",
                         "data": kwargs["short_term_memory"][self.data_loc] }}
        broker.publish(calib_note,"logged_note")
        return GratbotBehaviorStatus.COMPLETED, {}



def locate_object_neck_with_label(allowed_labels):
    return GratbotBehavior_Fallback([FocusOnObjectOfLabel(allowed_labels),ServoUpAndDown()])

def calibrate_neck_motion_step(labels,step_size):
    return GratbotBehavior_Checklist([
        GratbotBehavior_Series([FocusOnObjectOfLabel(labels),NoteInitialTrackPos(step_size)]),
        RunServoDelta(0,step_size),
        GratbotBehavior_Wait(2.0),
        GratbotBehavior_Series([FocusOnObjectOfLabel(labels),RecordFinalTrackPos(step_size)])])

def calibrate_turn_motion_step(labels,turn_speed,turn_duration):
    return GratbotBehavior_Checklist([
        GratbotBehavior_Series([FocusOnObjectOfLabel(labels),NoteInitialTrackPos([turn_speed,turn_duration])]),
        RunMotors(turn_speed,-turn_speed,turn_duration),
        GratbotBehavior_Wait(2.0),
        GratbotBehavior_Series([FocusOnObjectOfLabel(labels),RecordFinalTrackPos([turn_speed,turn_duration],mem_loc="turn_calib")])])

def calibrate_neck_motion():
    allowed_labels=["face","orange","sports ball"]
    servo_jumps=np.linspace(2,10,7)
    task_list=[]
    for j in servo_jumps:
        task_list.append(
            GratbotBehavior_Checklist([
                GratbotBehavior_Series([
                    locate_object_neck_with_label(allowed_labels),
                    NeckCenterFocusObject()]),
                GratbotBehavior_Wait(1.0),
                Announce("Moving up {}".format(j)),
                IgnoreFailure(calibrate_neck_motion_step(allowed_labels,j)),
                IgnoreFailure(calibrate_neck_motion_step(allowed_labels,j)),
                IgnoreFailure(calibrate_neck_motion_step(allowed_labels,-j)),
                IgnoreFailure(calibrate_neck_motion_step(allowed_labels,-j))]))
    task_list.append(Announce("Saving Neck Servo Calibration"))
    #task_list.append(BroadcastCalibration("neck_calib_dservo","neck_calib_track","neck_calib_rotvec"))
    task_list.append(BroadcastCalibration("neck_calib"))
    return DoOnce(GratbotBehavior_Checklist(task_list))

def calibrate_turn_motion_motors_only():
    allowed_labels=["face"]
    motor_speeds=np.linspace(0.2,1.0,5)
    task_list=[]
    motor_speed_times_time=0.1
    for j in motor_speeds:
        task_list.append(
            GratbotBehavior_Checklist([
                GratbotBehavior_Series([
                    locate_object_neck_with_label(allowed_labels),
                    Announce("Located object ")]),
                GratbotBehavior_Wait(1.0),
                Announce("Turning {}".format(j)),
                IgnoreFailure(calibrate_turn_motion_step(allowed_labels,j,motor_speed_times_time/j)),
                IgnoreFailure(calibrate_turn_motion_step(allowed_labels,-j,motor_speed_times_time/j))]))
    task_list.append(Announce("Saving Turn Motor Calibration"))
    task_list.append(BroadcastCalibration("turn_calib"))
    return DoOnce(GratbotBehavior_Checklist(task_list))

def calibrate_turn_motion():
    allowed_labels=["face","orange","sports ball"]
    servo_jumps=np.linspace(1,10,14)
    motor_speeds=np.linspace(0.2,1.0,5)
    motor_speed_times_time=0.1
    neck_poses=[0.2,0.5,0.8]
    task_list=[]
    for neck_pos in neck_poses:
        for j in motor_speeds:
            task_list.append(
                GratbotBehavior_Checklist([
                    GratbotBehavior_Series([
                        locate_object_neck_with_label(allowed_labels),
                        Announce("Adjusting Neck Position to {}".format(neck_pos)),
                        NeckCenterFocusObject(target_position=neck_pos),
                        Announce("Adjusting Turn Position"),
                        TurnCenterFocusObject()]),
                    GratbotBehavior_Wait(1.0),
                    Announce("Turning {}".format(j)),
                    IgnoreFailure(calibrate_turn_motion_step(allowed_labels,j,motor_speed_times_time/j)),
                    IgnoreFailure(calibrate_turn_motion_step(allowed_labels,-j,motor_speed_times_time/j))]))
    task_list.append(Announce("Saving Turn Motor Calibration"))
    #task_list.append(BroadcastCalibration("neck_calib_dservo","neck_calib_track","neck_calib_rotvec"))
    task_list.append(BroadcastCalibration("turn_calib"))
    return DoOnce(GratbotBehavior_Checklist(task_list))


#### Not updated below here

class TestCanSeeObject(GratbotBehavior):
    def __init__(self,memory_loc):
        self.memory_loc=memory_loc
    def act(self,**kwargs):
        if self.memory_loc not in kwargs["short_term_memory"]:
            return GratbotBehaviorStatus.FAILED
        if extract_track_with_id(self.to_track) is not None:
            return GratbotBehaviorStatus.COMPLETED
        return GratbotBehaviorStatus.FAILED



class SlowHeadCenterObject(GratbotBehavior):
    def __init__(self,angle):
        self.to_track=None
        self.success_error=0.1
        self.hardcode_step=4

    def act(self,**kwargs):
        track=extract_track_with_id(self.to_track)
        if track==None:
            return GratbotBehaviorStatus.FAILED
        center_y=track["center"][1]
        error=center_y-0.5
        if abs(error)<self.success_error:
            return GratbotBehaviorStatus.COMPLETED
        to_do=None
        if error>0:
           to_do=RunServoDelta(0,self.hardcode_step)
        else:
           to_do=RunServoDelta(0,-self.hardcode_step)
        to_do.act(kwargs)
        return GratbotBehaviorStatus.FAILED

class HeadSearchForObject(GratbotBehavior):
    def __init__(self,labels,memory_loc):
        self.labels_to_find=labels
        self.max_angle=0
        self.min_angle=0
        self.hardcode_step=5
        self.up_not_down=True
        self.memory_loc=memory_loc

    def act(self,**kwargs):
        track=extract_track_with_label(kwargs["short_term_memory"],self.labels_to_find)
        if track is not None:
            return GratbotBehaviorStatus.COMPLETED
        if self.up_not_down:
           to_do=RunServoDelta(0,self.hardcode_step)
           to_do.act(kwargs)
           return GratbotBehaviorStatus.INPROGRESS
        else:
           to_do=RunServoDelta(0,-self.hardcode_step)
           to_do.act(kwargs)
           return GratbotBehaviorStatus.INPROGRESS
        return GratbotBehaviorStatus.INPROGRESS

class CalibHeadServoWhileTracking(GratbotBehavior):
    def __init__(self):
        self.labels_to_track=labels
        self.to_track=None
        self.search_behavior=HeadSearchForObject(self.labels_to_track)
        self.track_behavior=SlowHeadCenterObject(None)
        self.sub_behavior=None

    def act(self,**kwargs):
        response=self.search_behavior.act(kwargs)
        if response is not GratbotBehaviorStatus.COMPLETED:
            return GratbotBehaviorStatus.INPROGRESS
        track=extract_track_with_label(kwargs["short_term_memory"],self.labels_to_track)
        if track==None:
            return GratbotBehaviorStatus.INPROGRESS
        self.track_behavior.to_track=track["id"]
        response=self.track_behavior.act(kwargs)
        if response is not GratbotBehaviorStatus.COMPLETED:
            return GratbotBehaviorStatus.INPROGRESS


class ExerciseServo(GratbotBehavior):
    def __init__(self):
        servo_min_angle=100
        servo_max_angle=160
        n_steps=12
        steps=np.concatenate([np.linspace(servo_min_angle,servo_max_angle,n_steps),np.linspace(servo_max_angle,servo_min_angle,n_steps)])
        motion_list=[]
        for s in steps:
            motion_list.append(self.wrap_motion_act(RunServo(0,s)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )


class ExerciseTurns(GratbotBehavior):
    def __init__(self):
        motion_list=[]
        turnmotion=np.linspace(np.pi/32,np.pi/2,4)
        for m in turnmotion:
            motion_list.append(self.wrap_motion_act(TurnRelative(m)))
            motion_list.append(self.wrap_motion_act(TurnRelative(-m)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehaviorFB(GratbotBehavior):

    def __init__(self):
        nsteps=6
        motion_list=[]
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[1.0]*nsteps
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehavior(GratbotBehavior):
    def __init__(self):
        nsteps=6
        motion_list=[]
        #aheadmotion=np.linspace(0.2,1.0,nsteps)
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[0.2]*nsteps
#        for i in range(nsteps): #fb
#            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
#            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],-aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],aheadmotion[i],aheaddur[i])))
        nsteps=5
        #aheadmotion=np.linspace(0.3,1.0,nsteps)
        aheadmotion=np.linspace(0.5,1.0,nsteps)
        aheaddur=[0.2]*nsteps
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],0.5*aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-0.5*aheadmotion[i],aheaddur[i])))
        for i in range(nsteps): #turn
            motion_list.append(self.wrap_motion_act(RunMotors(0.5*aheadmotion[i],aheadmotion[i],aheaddur[i])))
            motion_list.append(self.wrap_motion_act(RunMotors(-0.5*aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #aheaddur=[0.2]*nsteps
        #for i in range(nsteps): #fb
        #    motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],aheadmotion[i],aheaddur[i])))
        #    motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #for i in range(nsteps): #turn
        #    motion_list.append(self.wrap_motion_act(RunMotors(aheadmotion[i],-aheadmotion[i],aheaddur[i])))
        #    motion_list.append(self.wrap_motion_act(RunMotors(-aheadmotion[i],aheadmotion[i],aheaddur[i])))

        #ahead_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in ahead_motion:
        #    motion_list.append(self.wrap_motion_act(MoveAhead(am)))
        #turn_motion=[0.2,-0.2,0.4,-0.4,0.6,-0.6,0.8,-0.8]
        #for i in range(100):
        #     motion_list.append(self.wrap_motion_act(Turn(random.choice(turn_motion))))

        #for am in turn_motion:
        #    motion_list.append(self.wrap_motion_act(Turn(am)))
        #slew_motion=[0.2,-0.2,0.4,-0.4,0.5,-0.5]
        #for am in slew_motion:
        #    motion_list.append(self.wrap_motion_act(Slew(am)))
        self.action_list=GratbotBehavior_Series(motion_list)

    def wrap_motion_act(self,motion_act):
        return GratbotBehavior_Series([Announce("moving"),motion_act,GratbotBehavior_Wait(1.0)] )
        #return GratbotBehavior_Series([WaitForStablePose(),Announce("moving"),motion_act,GratbotBehavior_Wait(2.0),WaitForStablePose()])

    def act(self,**kwargs):
        return self.action_list.act(**kwargs)

class CalibrateMotionBehavior_WithTracking_Turns(GratbotBehavior):
    def __init__(self,labels):
        self.to_track=labels
        self.sub_behavior=None
        self.next_timestamp=0
        self.power_dur_product=0.15
        self.things_to_try=[ 0.4,0.5,0.6,0.7,0.8,0.9,1.0]
        #self.things_to_try=[ [0.4,0.5],[0.6,0.4],[0.7,0.3],[0.8,0.2]]
        self.on_thing_to_try=0

    def act(self,**kwargs):
        #if I'm in the middle of something, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.INPROGRESS:
                return step_status
            else:
                self.sub_behavior=None
            return GratbotBehaviorStatus.INPROGRESS

        #time to figure out something new to do
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            the_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            if the_timestamp<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info

            for track in tracks:
                logger.info("examining {}".format(track["label"]))
                if track["label"] in self.to_track:
                    #I presume its holding still here
                    error=track["center"][0]-0.5
                    #if I'm too far away from centered, recenter on object
                    if abs(error)>0.1:
                        logger.info("correcting")
                        x=np.sign(error)*0.4
                        todo=RunMotors(x,-x,0.1)
                        self.next_timestamp=the_timestamp+0.2
                        todo.act(**kwargs)
                        return GratbotBehaviorStatus.INPROGRESS
                    #Now try one of those other actions
                    elem=self.things_to_try[self.on_thing_to_try]
                    dur=self.power_dur_product/elem
                    logger.info("taking a turn {} , {}".format(elem,dur))
                    self.sub_behavior=self.wrap_motion_act(RunMotors(elem,-elem,dur),RunMotors(-elem,elem,dur))
                    self.on_thing_to_try+=1
                    if self.on_thing_to_try>=len(self.things_to_try):
                        self.on_thing_to_try=0
                    return GratbotBehaviorStatus.INPROGRESS
                logger.info("ignoring {}".format(track["label"]))

        #if I can't find the object I'm tracking, just wait
        logger.info("Can't find what I'm looking for.  Waiting. ")
        return GratbotBehaviorStatus.INPROGRESS

    def wrap_motion_act(self,motion_act1,motion_act2):
        return GratbotBehavior_Series([motion_act1,GratbotBehavior_Wait(1.0),motion_act2,GratbotBehavior_Wait(1.0)] )



class CalibrateMotionBehavior_WithTracking_FB(GratbotBehavior):
    def __init__(self,labels):
        self.to_track=labels
        self.sub_behavior=None
        self.next_timestamp=0
        self.power_dur_product=1.2
        self.things_to_try=[ 0.4,0.5,0.6,0.7,0.8,0.9,1.0]
        #self.things_to_try=[ [0.4,0.5],[0.6,0.4],[0.7,0.3],[0.8,0.2]]
        self.on_thing_to_try=0

    def act(self,**kwargs):
        #if I'm in the middle of something, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.INPROGRESS:
                return step_status
            else:
                self.sub_behavior=None
            return GratbotBehaviorStatus.INPROGRESS

        #time to figure out something new to do
        if "tracks" in kwargs["short_term_memory"]:
            tracks=kwargs["short_term_memory"]["tracks"]["tracks"]
            the_timestamp=kwargs["short_term_memory"]["tracks"]["timestamp"]
            if the_timestamp<=self.next_timestamp:
                return GratbotBehaviorStatus.INPROGRESS #wait for new tracks info

            for track in tracks:
                logger.info("examining {}".format(track["label"]))
                if track["label"] in self.to_track:
                    #I presume its holding still here
                    error=track["center"][0]-0.5
                    #if I'm too far away from centered, recenter on object
                    if abs(error)>0.1:
                        logger.info("correcting")
                        x=np.sign(error)*0.4
                        todo=RunMotors(x,-x,0.1)
                        self.next_timestamp=the_timestamp+0.2
                        todo.act(**kwargs)
                        return GratbotBehaviorStatus.INPROGRESS
                    #Now try one of those other actions
                    elem=self.things_to_try[self.on_thing_to_try]
                    dur=self.power_dur_product/elem
                    logger.info("taking a fb  {} , {}".format(elem,dur))
                    self.sub_behavior=self.wrap_motion_act(RunMotors(elem,elem,dur),RunMotors(-elem,-elem,dur),dur)
                    self.on_thing_to_try+=1
                    if self.on_thing_to_try>=len(self.things_to_try):
                        self.on_thing_to_try=0
                    return GratbotBehaviorStatus.INPROGRESS
                logger.info("ignoring {}".format(track["label"]))

        #if I can't find the object I'm tracking, just wait
        logger.info("Can't find what I'm looking for.  Waiting. ")
        return GratbotBehaviorStatus.INPROGRESS

    def wrap_motion_act(self,motion_act1,motion_act2,extra_wait):
        return GratbotBehavior_Series([motion_act1,GratbotBehavior_Wait(1.0+extra_wait),motion_act2,GratbotBehavior_Wait(1.0+extra_wait)] )
