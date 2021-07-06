
from Behavior import GratbotBehavior
from Behavior import GratbotBehaviorStatus
from gyrii.underpinnings.GratbotLogger import gprint,gprint_low
from underpinnings.BayesianArray import BayesianArray
from collections import deque
import numpy as np
import time
import random
import json

def first_object_with_property_in_list(thelist,theproperty,thevalue):
    for obj in thelist:
        if obj[theproperty]==thevalue:
            return obj
    return None

class CalibrateChasingBehavior(GratbotBehavior):
    def __init__(self):
        self.objects_to_watch=["sports ball","stop sign","chair"]
        self.check_in_period=0.1
        self.last_check=0
        self.screen_center=320

    def act(self,**kwargs):
        #only act so often
        if time.time()<self.last_check+self.check_in_period:
            return GratbotBehaviorStatus.INPROGRESS #only act every so often
        self.last_check=time.time()

        #only act if I'm not moving
        gyro_z=kwargs["short_term_memory"]["position_sensor/gyro"]["position_sensor/gyro"][2]
        gyro_z_stdev=kwargs["short_term_memory"]["position_sensor/gyro"]["position_sensor/gyro_stdev"][2]
        if abs(gyro_z)>0.05 or gyro_z_stdev>0.05:
            gprint("moving, not acting")
            return GratbotBehaviorStatus.INPROGRESS #still turning

        broker=kwargs["broker"]
        visual_tracker_objects=kwargs["short_term_memory"]["tagged_objects"]["tagged_objects"]
        labels=[ x["label"] for x in visual_tracker_objects ]
        if len(set(labels).intersection(set(self.objects_to_watch)))==0:
            #If I don't have anything to watch, just start turning
            gprint("nothing to see, turning")
            broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [0.6,-0.6], "duration":self.check_in_period*1.1 } },"motor_command")

        for object_type in self.objects_to_watch:
            found_obj=first_object_with_property_in_list(visual_tracker_objects,"label",object_type)
            if found_obj==None: #there are objects, but not of the type I'm looking for
                continue
            actual_x=0.5*(found_obj["startx"]+found_obj["endx"])
            #turn_mag=random.uniform(0.2,0.6)
            a=random.uniform(0.2,0.8)
            rand_dir=1
            if random.uniform(0,1)>0.5:
                rand_dir=-1
            run_dur=random.uniform(0.05,0.2)
            #turn_mag=random.uniform(0.2,0.6)
            #turn_dur=random.uniform(0.05,0.2)
            #turn_dir=1
            #if actual_x<self.screen_center:
            #    turn_dir=-1
            #lt=turn_mag*turn_dir
            #rt=-turn_mag*turn_dir
            #if actual_x<self.screen_center:
            #    lt=min(a,b)
            #    rt=max(a,b)
            #else:
            #    lt=max(a,b)
            #    rt=min(a,b)
            lt=a*rand_dir
            rt=a*rand_dir

            broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration":run_dur } },"motor_command")
            self.last_check=time.time()+run_dur
            return GratbotBehaviorStatus.INPROGRESS
        gprint("error couldn't find object")

        return GratbotBehaviorStatus.INPROGRESS


class TrackWithCameraBehavior(GratbotBehavior):
    def __init__(self,object_to_track):
        self.object_to_track=object_to_track
        self.last_processed_timestamp=0

    def act(self,**kwargs):
        broker=kwargs["broker"]
        if "visual_tracker_objects" not in kwargs["short_term_memory"]:
            gprint("No objects to track")
            return GratbotBehaviorStatus.FAILED
        visual_tracker_objects=kwargs["short_term_memory"]["visual_tracker_objects"]["visual_tracker_objects"]
        visual_tracker_objects_timestamp=kwargs["short_term_memory"]["visual_tracker_objects"]["timestamp"]
        if visual_tracker_objects_timestamp-self.last_processed_timestamp<0.1:
            return GratbotBehaviorStatus.INPROGRESS #give it time for last command to process
        found_obj=first_object_with_property_in_list(visual_tracker_objects,"id",self.object_to_track)
        if found_obj==None:
            gprint("object lost")
            return GratbotBehaviorStatus.FAILED
        desired_x=320
        actual_x=found_obj["xywh"][0]
        x_velocity=found_obj["vxvyvwvh"][0]
        #gprint("actual x, xvel: {} {}".format(actual_x,x_velocity))
        #look to where it will be in the future
        projection_time=0.1
        future_x=actual_x+projection_time*x_velocity
        #gprint("futnure x {}".format(future_x))
        #gprint("desired x {}".format(desired_x))
        #TODO make this elegant, turn towards the thing
        error_x=future_x-desired_x
        #gprint("error x {}".format(error_x))
        if abs(error_x)<10:
            gprint("DONE")
            return GratbotBehaviorStatus.COMPLETED
        camera_hfov_pixels=(48.8*(2*np.pi)/360)/640 #from spec sheet
        turn_amount=float(error_x*camera_hfov_pixels)
        #gprint("turn_amount {}".format(turn_amount))
        broker.publish({"timestamp": time.time(),"move_command": {"type": "turn","angle": turn_amount}},"move_command")
        self.last_processed_timestamp=visual_tracker_objects_timestamp
        return GratbotBehaviorStatus.INPROGRESS

class TrackIfSeenBehavior(GratbotBehavior):
    def __init__(self,object_type):
        self.object_type=object_type
        self.sub_behavior=None

    def act(self,**kwargs):
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
        else:
            step_status=GratbotBehaviorStatus.COMPLETED
        if step_status==GratbotBehaviorStatus.COMPLETED or step_status==GratbotBehaviorStatus.FAILED:
            #if "visual_tracker_objects" not in kwargs["short_term_memory"]:
            if "tagged_objects" not in kwargs["short_term_memory"]:
                gprint("no objects")
                return GratbotBehaviorStatus.INPROGRESS
            #visual_tracker_objects=kwargs["short_term_memory"]["visual_tracker_objects"]["visual_tracker_objects"]
            visual_tracker_objects=kwargs["short_term_memory"]["tagged_objects"]["tagged_objects"]
            found_obj=first_object_with_property_in_list(visual_tracker_objects,"label",self.object_type)
            if found_obj==None: #there are objects, but not of the type I'm looking for
                gprint("no objects with rigth label {}".format(self.object_type))
                return GratbotBehaviorStatus.INPROGRESS
            #at this point I've seen an object of the desired type.  Track it
            #gprint("Object {} found, should track it!".format(found_obj["id"]))
            gprint("Object {} found, should track it!".format(found_obj["label"]))
            #self.sub_behavior=TrackWithCameraBehavior(found_obj["id"])
            #self.sub_behavior=PIDWatchBehavior(found_obj["id"])
            #self.sub_behavior=JustTurnBehavior()
            #self.sub_behavior=TurnWhileWatchingBehavior(found_obj["id"])
            self.sub_behavior=ChaseBehavior(found_obj["label"])

            return GratbotBehaviorStatus.INPROGRESS
        return GratbotBehaviorStatus.INPROGRESS

class ChaseBehavior(GratbotBehavior):
    def __init__(self,label):
        self.target_label=label
        self.camera_focal_length_pixels=630 #V1 raspicam
        self.target_angle=0
        self.target_dist=1.0
        self.camera_x_pixels=640
        self.camera_y_pixels=480
        self.object_heights={ "stop sign": [0.081,0.005], "sports ball": [0.115,0.01], "chair": [1.0,0.5]}
        self.last_command_time=0
        self.command_interval=0.2



    def act(self,**kwargs):
        if time.time()<self.last_command_time+self.command_interval:
            return GratbotBehaviorStatus.INPROGRESS


        broker=kwargs["broker"]
        if "tagged_objects" not in kwargs["short_term_memory"]:
            gprint("No objects to track")
            return GratbotBehaviorStatus.FAILED

        visual_tracker_objects=kwargs["short_term_memory"]["tagged_objects"]["tagged_objects"]
        visual_tracker_objects_timestamp=kwargs["short_term_memory"]["tagged_objects"]["timestamp"]
        found_obj=first_object_with_property_in_list(visual_tracker_objects,"label",self.target_label)
        if found_obj is not None:
            self.tracking_object=True
            x=0.5*(found_obj["startx"]+found_obj["endx"])
            height=(found_obj["endy"]-found_obj["starty"])
            angle=(x-self.camera_x_pixels/2)/self.camera_focal_length_pixels
            obj_height=self.object_heights[self.target_label][0]
            obj_dist=obj_height*self.camera_focal_length_pixels/height
            delta_angle=self.target_angle-angle
            delta_dist=self.target_dist-obj_dist
            if abs(delta_angle)<0.03 and abs(delta_dist)<0.1:
                return GratbotBehaviorStatus.COMPLETED
            broker.publish({"timestamp": time.time(),"visual_motion_request": {"from_angle": angle,"from_dist": obj_dist, "delta_angle": delta_angle,"delta_dist": delta_dist} },"visual_motion_request")
            self.last_command_time=time.time()
            return GratbotBehaviorStatus.INPROGRESS

        gprint("Lost Object of label ")
        return GratbotBehaviorStatus.FAILED

class JustTurnBehavior(GratbotBehavior):
    def __init__(self):
        self.change_freq=5.0
        self.last_processed_timestamp=0
        self.check_frequency=0.1
        self.last_change=0

    def update_turn_speed(self):
        self.turn_mag=random.uniform(0.4,0.6)
        self.turn_dir=random.choice([-1,1])
        self.last_change=time.time()

    def act(self,**kwargs):
        broker=kwargs["broker"]
        if time.time()-self.last_processed_timestamp<self.check_frequency:
            return GratbotBehaviorStatus.INPROGRESS #give it time for last command to process
        if time.time()-self.last_change>self.change_freq:
            self.update_turn_speed()
        lt=self.turn_mag*self.turn_dir
        rt=-self.turn_mag*self.turn_dir
        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration":self.check_frequency*1.1 } },"motor_command")
        self.last_processed_timestamp=time.time()
        return GratbotBehaviorStatus.INPROGRESS #give it time for last command to process

class TurnWhileWatchingBehavior(GratbotBehavior):
    def __init__(self,object_type):
        #self.object_to_track=object_to_track
        self.object_type=object_type
        self.last_processed_timestamp=0
        self.check_frequency=0.1
        self.max_width=640
        self.visual_range=0.9
        self.direction=1
        self.speeds=[0.1,0.2,0.3]
        self.on_speed=1
        self.last_speed_change=time.time()
        self.speed=0.1
        self.last_direction_change=0
        self.tracking_object=True
        self.speed_change=20.0

    def act(self,**kwargs):
        broker=kwargs["broker"]
        if "tagged_objects" not in kwargs["short_term_memory"]:
            gprint("No objects to track")
            return GratbotBehaviorStatus.FAILED
        visual_tracker_objects=kwargs["short_term_memory"]["tagged_objects"]["tagged_objects"]
        visual_tracker_objects_timestamp=kwargs["short_term_memory"]["tagged_objects"]["timestamp"]

        #found_obj=first_object_with_property_in_list(visual_tracker_objects,"id",self.object_to_track)
        found_obj=first_object_with_property_in_list(visual_tracker_objects,"label",self.object_type)
        #if found_obj==None:
        #    gprint("object lost")
        #    if self.tracking_object:
        #        self.direction=-self.direction
        #        self.last_direction_change=time.time()
        #    self.tracking_object=False
        #else:
        #    gprint("object found")
        if found_obj is not None:
            self.tracking_object=True
            #actual_x=found_obj["xywh"][0]
            actual_x=0.5*(found_obj["startx"]+found_obj["endx"])
            #gprint("actual x {} is it between {} and {}".format(actual_x,(1-self.visual_range)*self.max_width,self.visual_range*self.max_width))
            if actual_x<(1-self.visual_range)*self.max_width or actual_x>self.visual_range*self.max_width:
                if time.time()-self.last_direction_change>1.0:
                    self.direction=-self.direction
                    self.last_direction_change=time.time()

        if time.time()-self.last_speed_change>self.speed_change:
            self.on_speed+=1
            if self.on_speed>=len(self.speeds):
                self.on_speed=0
            gprint("turn speed now {}".format(self.speed))
            self.speed=self.speeds[self.on_speed]
            self.last_speed_change=time.time()

        if visual_tracker_objects_timestamp-self.last_processed_timestamp<self.check_frequency:
            return GratbotBehaviorStatus.INPROGRESS #give it time for last command to process

        lt=self.speed*self.direction
        rt=-self.speed*self.direction
        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration":self.check_frequency*1.1 } },"motor_command")
        return GratbotBehaviorStatus.INPROGRESS

class PIDWatchBehavior(GratbotBehavior):
    def __init__(self,object_to_track):
        self.object_to_track=object_to_track
        self.last_processed_timestamp=0
        self.check_frequency=0.1
        self.target_x=320

        self.error_history=deque([],5) #max length is 5
        self.last_prop=0
        self.last_inte=0
        self.last_diff=0
        self.data_log=[]
        self.last_lminusr=0

        self.last_change=0
        self.last_xvel=0

    def save_data_log(self):
        if len(self.data_log)<2:
            return
        f=open("tracker_data_log.txt",'a')
        f.write(json.dumps(self.data_log)+"\n")
        #for i in range(len(self.data_log)):
        #    outp=""
        #    for j in range(len(self.data_log[i])):
        #        outp+="{} ".format(self.data_log[i][j])
        #    f.write("{}\n".format(outp))
        f.close()

    def update_random_pid(self):
        self.pid_p=random.uniform(0.001,0.006)
        #self.pid_i=random.uniform(-0.002,0.002)
        self.pid_i=0
        #self.pid_d=random.uniform(-0.002,0.002)
        self.pid_d=0
        self.last_change=time.time()

    def act(self,**kwargs):
        broker=kwargs["broker"]
        if "visual_tracker_objects" not in kwargs["short_term_memory"]:
            gprint("No objects to track")
            return GratbotBehaviorStatus.FAILED
        visual_tracker_objects=kwargs["short_term_memory"]["visual_tracker_objects"]["visual_tracker_objects"]
        visual_tracker_objects_timestamp=kwargs["short_term_memory"]["visual_tracker_objects"]["timestamp"]
        if visual_tracker_objects_timestamp-self.last_processed_timestamp<self.check_frequency:
            return GratbotBehaviorStatus.INPROGRESS #give it time for last command to process
        found_obj=first_object_with_property_in_list(visual_tracker_objects,"id",self.object_to_track)
        if found_obj==None:
            gprint("object lost")
            return GratbotBehaviorStatus.FAILED
        if time.time()-self.last_change>1.0:
            self.update_random_pid()
        desired_x=320
        actual_x=found_obj["xywh"][0]
        x_velocity=found_obj["vxvyvwvh"][0]

        #TODO my projection time could be another parameter
        projection_time=0.1
        #future_x=actual_x+projection_time*x_velocity
        error_x=actual_x-desired_x



        prop=error_x
        diff=0
        if len(self.error_history)>0:
            diff=error_x-self.error_history[0]
        inte=(error_x+np.sum(self.error_history))/(len(self.error_history)+1)

        #record how things went before
        if len(self.error_history)>0:
            #p,i,d, prop,diff,int, new error
            self.data_log.append([self.pid_p,self.pid_i,self.pid_d,self.last_prop,self.last_inte,self.last_diff,self.last_lminusr,self.last_xvel,error_x])


        lminusr=self.pid_p*prop+self.pid_i*inte+self.pid_d*diff
        gprint("lminus r {}".format(lminusr))
        lminusr_clipped=np.clip(lminusr,-1,1)
        if abs(lminusr_clipped)<0.4:
            lminusr_clipped=0
        #gprint("lminus r clipped {}".format(lminusr_clipped))
        if self.last_lminusr==0 and lminusr_clipped==0:
            self.save_data_log()
            return GratbotBehaviorStatus.COMPLETED

        lt=lminusr_clipped
        rt=-lminusr_clipped

        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration":self.check_frequency*1.1 } },"motor_command")

        self.error_history.appendleft(error_x)
        self.last_prop=prop
        self.last_inte=inte
        self.last_diff=diff
        self.last_lminusr=lminusr_clipped
        self.last_xvel=x_velocity
        return GratbotBehaviorStatus.INPROGRESS
