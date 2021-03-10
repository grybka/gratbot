
from rolly_behaviors import DisplayCamera
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np
import sys
sys.path.append('../yolov5_tool')
from yolov5_tool import yolov5_tool

class RollyChase(DisplayCamera):
    def __init__(self,comms):
        super().__init__(comms)
        self.yv5model=None
        self.wait_duration_seconds=1
        self.next_action_time=time.time()+self.wait_duration_seconds
        self.video_objects=None
        self.yv5model=None
        self.tracking=False

    def get_top_object(self,video_objects,object_class,min_confidence,video_frame):
        best_obj=None
        for obj in video_objects:
            if obj["label"]==object_class and obj["confidence"]>min_confidence:
                if best_obj==None:
                    best_obj=obj
                else:
                    if obj["confidence"]>best_obj["confidence"]:
                        best_obj=obj
        if best_obj==None:
            if self.tracking==True:
                #this means I lost it
                cv.imwrite(time.strftime("%Y%m%d-%H%M%S_lost.jpg"),video_frame)
                self.tracking=False
            return None
        self.tracking=True

        return best_obj

    def tag_objects(self,video_frame):
        if self.yv5model==None:
            self.yv5model=yolov5_tool()
                #self.yv5model.initialize("E:/projects/yolov5/runs/exp9/weights/last.pt")
            self.yv5model.initialize("C:/Users/grybk/projects/yolov5/yolov5/runs/exp28/weights/last.pt")
        video_objects=self.yv5model.detect(video_frame)
        self.yv5model.draw_object_bboxes(video_frame,video_objects)
        return video_objects,video_frame


    def get_face_loc_width(self,face):
        centerx=0.5*(face["startx"]+face["endx"])
        centery=0.5*(face["starty"]+face["endy"])
        width=(-face["startx"]+face["endx"])
        height=(-face["starty"]+face["endy"])
        return np.array([centerx,centery]),np.array([width,height])

    def act_on_objects(self,to_track):
        xy,wh=self.get_face_loc_width(to_track)

        video_height=480
        video_width=640
        dx=video_width/2-xy[0] #how does it know the size
        dy=video_height/2-xy[1]
        #if it's up or down, tell the camera to move
        camera_pixel_to_servo=0.1
        dy_servo=dy*camera_pixel_to_servo
        #self.comms.set_intention( ["camera_x","position_delta","SET" ], dx_servo )
        if abs(dy_servo)>5:
            self.comms.set_intention( ["camera_pitch_servo","position_delta","SET" ], dy_servo)
            print("camera {}".format(dy_servo))

        #if its left or right, turn the wheels
        wheel_pixel_to_servo=1/200.0
        dx_wheels=dx*wheel_pixel_to_servo
        print("wheels {}".format(dx_wheels))
        if abs(dx_wheels)>0.1:
            self.comms.set_intention( ["wheel_turn_servo","position","SET" ], dx_wheels )

        #if it's left or right, tell the legs to move
        right_size_width=170
        speed_scale=2.5
        wheel_motion_clip=0.7
        wheel_motion_fb=np.clip(-speed_scale*( 1-right_size_width/wh[0]),-wheel_motion_clip,wheel_motion_clip)
        if abs(wheel_motion_fb)>0.2:
            self.comms.set_intention( ["wheel_motor","speed","SET" ], 100*wheel_motion_fb )
            print("wheel motion {}".format(wheel_motion_fb))
        else:
            self.comms.set_intention( ["wheel_motor","stop","SET" ], 1)


    def act(self):
        image=self.get_image()
        if image is not None: #run decide for each new image
            video_objects,image=self.tag_objects(image)
            to_track=self.get_top_object(video_objects,"purple_ball",0.2,image)
            if to_track is not None:
                self.act_on_objects(to_track)
            return image
        return None
