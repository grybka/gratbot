
from rolly_behaviors import DisplayCamera
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np

class RollyChase(DisplayCamera):
    def __init__(self,comms):
        super().__init__(comms)
        self.yv5model=None
        self.wait_duration_seconds=1
        self.next_action_time=time.time()+self.wait_duration_seconds
        self.video_objects=None

    def get_top_object(self,video_objects,object_class,min_confidence):
        best_obj=None
        for obj in video_objects:
            if obj["label"]==object_class and obj["confidence"]>min_confidence:
                if best_obj==None:
                    best_obj=obj
                else:
                    if obj["confidence"]>best_obj["confidence"]:
                        best_obj=obj
        return best_obj

    def tag_objects(self,video_frame):
        if self.yv5model==None:
            self.yv5model=yolov5_tool()
                #self.yv5model.initialize("E:/projects/yolov5/runs/exp9/weights/last.pt")
            self.yv5model.initialize("C:/Users/grybk/projects/yolov5/yolov5/runs/exp28/weights/last.pt")
        video_objects=get_video_frame_objects_yolov5(video_frame,self.yv5model)
        draw_object_bboxes(video_frame,video_objects)
        return video_objects,video_frame

    def act_on_objects(self,to_track):
        to_track=None
        to_track=self.get_top_object(video_objects,"purple_ball",0.2)
        if to_track==None:
            if self.tracking==True:
                #this means I lost it
                self.tracking=False
            return
        self.tracking=True
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
            self.actions.append( [ ["camera_yaw_servo","position_delta","SET" ], dy_servo ])
            print("camera {}".format(dy_servo))

        #if its left or right, turn the wheels
        wheel_pixel_to_servo=1/300.0
        dx_wheels=dy*wheel_pixel_to_servo

        #if it's left or right, tell the legs to move
        pixel_to_leg=1./320.
        leg_motion_turn=np.clip(dx*pixel_to_leg,-1,1)
        right_size_width=170
        speed_scale=2.0
        leg_motion_fb=np.clip(-speed_scale*( 1-right_size_width/wh[0]),-1,1)
        leg_motion_r=np.clip(leg_motion_fb+leg_motion_turn,-1,1)
        leg_motion_l=np.clip(leg_motion_fb-leg_motion_turn,-1,1)
        print("leg_motion_fb {}".format(leg_motion_fb))
        print("leg_motion_turn {}".format(leg_motion_turn))
        if abs(leg_motion_r)>0.15 or abs(leg_motion_l)>0.15:
            self.actions.append([ [ "leg_controller","on_off", "SET" ], 1])
            self.actions.append([ [ "leg_controller","left_speed", "SET" ], leg_motion_l])
            self.actions.append([ [ "leg_controller","right_speed", "SET" ], leg_motion_r])

    def act(self):
        image=self.get_image()
        if image is not None: #run decide for each new image
            video_objects,image=self.tag_objects(image)
            to_track=self.get_top_object(video_objects,"purple_ball",0.2)
            #cv.imwrite(time.strftime("%Y%m%d-%H%M%S.jpg"),video_frame)
            if to_track is not None:
                self.act_on_objects(to_track)
            return image
        return None
