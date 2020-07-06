# Behaviors for hexapod

import numpy as np
import time
import filterpy
import random

import cv2 as cv
import cvlib
#from cvlib.object_detection import draw_bbox

#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise

def get_video_frame_faces(video_frame):
    video_objects=[]
    faces,confidences=cvlib.detect_face(video_frame)
    for i in range(len(faces)):
        face=faces[i]
        video_objects.append({
            "confidence": confidences[i],
            "label": "face",
            "startx": face[0],
            "starty": face[1],
            "endx": face[2],
            "endy": face[3]
        })
    return video_objects
#                confidence=confidences[i]
#                (startx,starty)=face[0],face[1]
#                (endx,endy)=face[2],face[3]
#                cv.rectangle(myframe,(startx,starty),(endx,endy),(0,255,0),2)
#                text = "Face {:.2f}%".format(confidence * 100)
#                Y = starty - 10 if starty - 10 > 10 else starty + 10
#                # write confidence percentage on top of face rectangle
#                cv.putText(myframe, text, (startx,Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
            #draw_bbox(myframe,bbox,label,conf,write_conf=True)

def get_video_frame_objects(video_frame,video_objects=None):
    if video_objects==None:
        video_objects=[]
    bbox,label,conf=cvlib.detect_common_objects(video_frame,confidence=0.4,model='yolov3-tiny')
    for i in range(len(label)):
        box=bbox[i]
        video_objects.append({
            "confidence": conf[i],
            "label": label[i],
            "startx": box[0],
            "starty": box[1],
            "endx": box[2],
            "endy": box[3]
        })
    return video_objects

def draw_object_bboxes(video_frame,video_objects):
    for i in range(len(video_objects)):
        startx=video_objects[i]["startx"]
        starty=video_objects[i]["starty"]
        endx=video_objects[i]["endx"]
        endy=video_objects[i]["endy"]
        confidence=video_objects[i]["confidence"]
        cv.rectangle(video_frame,(startx,starty),(endx,endy),(0,255,0),2)
        text = "{} {:.2f}%".format(video_objects[i]["label"],confidence * 100)
        Y = starty - 10 if starty - 10 > 10 else starty + 10
        cv.putText(video_frame, text, (startx,Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)

class GratbotBehavior:
    def __init__(self, comms):
        self.comms = comms
    def act(self,video_frame):
        return


class JustSaveObjectPos(GratbotBehavior):
    def __init__(self,comms):
        super().__init__(comms)
        self.move_duration_seconds=1
        self.wait_duration_seconds=1
        self.next_action_time=time.time()+self.wait_duration_seconds

    def get_face_loc_width(self,face):
        centerx=0.5*(face["startx"]+face["endx"])
        centery=0.5*(face["starty"]+face["endy"])
        width=(-face["startx"]+face["endx"])
        height=(-face["starty"]+face["endy"])
        return np.array([centerx,centery]),np.array([width,height])

    def act(self, video_frame):
        now=time.time()
        if now>self.next_action_time:
            self.next_action_time=now+self.move_duration_seconds
            video_objects=get_video_frame_faces(video_frame)
            video_objects=get_video_frame_objects(video_frame,video_objects)
            for obj in video_objects:
                if obj["label"]=="face":
                    loc,ext=self.get_face_loc_width(obj)
                    print("Face {} {}".format(ext[0],ext[1]))
                if obj["label"]=="person":
                    loc,ext=self.get_face_loc_width(obj)
                    print("Person {} {}".format(ext[0],ext[1]))
            draw_object_bboxes(video_frame,video_objects)
            return video_frame
        return None



##------ Won't work belowe here----



#class GratbotBehavior:
#    def __init__(self, comms):
#        self.comms = comms
#    def act(self,video_objects):
#        return

class WaveAtFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.waving=False
    def act(self,video_objects):
        wave_controller="wave_paw"
        idle_controller="legs_idle"
        #TODO Fix this to accomodate that video bojects now have label of face
        if self.waving==False and "faces" in video_objects:
            self.waving=True
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 0 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"left_speed","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"right_speed","SET" ], 1 )
        elif self.waving==True and "faces" not in video_objects:
            self.waving=False
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 0 )


class HeadTrackFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
#        self.filter_y=KalmanFilter(dim_x=2,dim_z=1)
#        self.filter_y.x=np.array([2.,0.]) #pos, vel
#        self.filter_y.F=np.array([[1.0,1.0],[0.,1.]]) #x=x0+vt
#        self.filter_y.P *= 1000. #covariance, not sue
#        self.filter_y.H =np.array([ [1.0,0]])
#        self.filter_y.R = np.array([[10]]) #measurement noise
#        self.filter_y.Q=Q_discrete_white_noise(dim=2,dt=0.1,var=0.13)
#
#        self.filter_x=KalmanFilter(dim_x=2,dim_z=1)
#        self.filter_x.x=np.array([2.,0.]) #pos, vel
#        self.filter_x.F=np.array([[1.0,1.0],[0.,1.]]) #x=x0+vt
#        self.filter_x.P *= 1000. #covariance, not sue
#        self.filter_x.H =np.array([ [1.0,0]])
#        self.filter_x.R = np.array([[10]]) #measurement noise
#        self.filter_x.Q=Q_discrete_white_noise(dim=2,dt=0.1,var=0.13)
        self.last_time=time.time()
    def act(self, video_objects):
        now=time.time()
        act_often=0.1
        if now-self.last_time < act_often:
            return
        self.last_time=now
        video_height=480
        video_width=640
        pixel_to_servo=0.1
        for x in video_objects:
            if x["label"]=="face":
                face=x
                centerx=0.5*(face["startx"]+face["endx"])
                centery=0.5*(face["starty"]+face["endy"])
                print("face at {}, {}".format(centerx,centery))
                dx=video_width/2-centerx #how does it know the size
                dy=video_height/2-centery
#            self.filter_y.predict()
#            self.filter_x.predict()
#            self.filter_y.update([dy])
#            self.filter_x.update([dx])
#            dy=self.filter_y.x[0]
#            dx=self.filter_x.x[0]
                print("dy updated {}".format(dy))
                dx_servo=dx*pixel_to_servo
                dy_servo=dy*pixel_to_servo
                #print("move {},{}".format(dx_servo,dy_servo))
                #print("move {}".format(dy_servo))
                self.comms.set_intention( ["camera_x","position_delta","SET" ], dx_servo )
                self.comms.set_intention( ["camera_y","position_delta","SET" ], dy_servo )
                break
            #self.filter_y.predict()
            #self.comms.set_intention( ["camera_x","position","SET" ], 0 )
            #self.comms.set_intention( ["camera_y","position","SET" ], 0 )

#TODO Make state machine
class NondeterministicStateMachine():
    def __init__(self):
        self.on_state="start"
        self.transitions={}

    def advance():
        transition_names=self.transitions[self.on_state]["transition_names"]
        transition_weights=self.transitions[self.on_state]["transition_weights"]
        self.on_state=random.choice(transition_names,transition_weights)

class MoveAndTrackObjects(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.state="holding_still"
        self.move_duration_seconds=1
        self.wait_duration_seconds=1
        self.next_action_time=time.time()+self.wait_duration_seconds
        self.old_video_objects=[]
        self.step_history=[]
        self.last_move_choice=np.array([0.0,0.0])
        #self.legs_or_camera="camera"
        self.legs_or_camera="legs"
        self.moved_back=True
    def get_face_loc_width(self,face):
        centerx=0.5*(face["startx"]+face["endx"])
        centery=0.5*(face["starty"]+face["endy"])
        width=(-face["startx"]+face["endx"])
        height=(-face["starty"]+face["endy"])
        return np.array([centerx,centery]),np.array([width,height])
    def capture(self,video_objects):
        #for each object in old video video_objects
        #if exacly one of them exists in new video objects, assume it is the same one
        #print("old video objects {}".format(self.old_video_objects))
        #print("video objects {}".format(video_objects))
        for old_obj in self.old_video_objects:
            for obj in video_objects:
                if old_obj["label"]==obj["label"]:
                    old_loc,old_ext=self.get_face_loc_width(old_obj)
                    loc,ext=self.get_face_loc_width(obj)
                    step_desc=np.concatenate([self.last_move_choice,old_loc,old_ext,loc,ext,[obj["label"]]])
                    self.step_history.append(step_desc)
                    outfile=open("movelist.txt","a+")
                    for i in range(len(step_desc)):
                        outfile.write("{} ".format(step_desc[i]))
                    outfile.write("\n")
                    outfile.close()
                    #print(self.step_history[-1])
        self.old_video_objects=video_objects
    def act(self, video_objects):
        #hold still
        #record where objects are
        #choose a combination for walking
        now=time.time()
        if now>self.next_action_time:
            print("state {}".format(self.state))
            if self.state=="holding_still":
                #capture a motion
                self.capture(video_objects)
                #begin a move
                #decide new random move here
                if self.moved_back:
                    self.last_move_choice=np.array([random.uniform(-1,1),random.uniform(-1,1)])
                    print("move choice {}".format(self.last_move_choice))
                    self.moved_back=False
                else:
                    self.last_move_choice=-self.last_move_choice
                    self.moved_back=True
                print("move {}".format(self.last_move_choice))
                if self.legs_or_camera=="legs":
                    self.comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                    self.comms.set_intention( [ "leg_controller","left_speed", "SET" ], self.last_move_choice[0])
                    self.comms.set_intention( [ "leg_controller","right_speed", "SET" ], self.last_move_choice[1])
                else:
                    self.comms.set_intention( ["camera_x","position_delta","SET" ], self.last_move_choice[0]*10)
                    self.comms.set_intention( ["camera_y","position_delta","SET" ], self.last_move_choice[1]*10)
                self.state="moving"
                self.next_action_time=now+self.move_duration_seconds
            elif self.state=="moving":
                if self.legs_or_camera=="legs":
                    self.comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
                    print("stopping")
                self.state="holding_still"
                self.next_action_time=now+self.wait_duration_seconds



class FollowFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
    #def act(self, video_objects):
        #identify if there are any faces in view
        #if so, calculate the difference between where the face is and where i wnant it to be
        #feed the difference into my move predictor
