import sys
import yaml
import time
import logging
import traceback
import numpy as np
import cv2 as cv
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotMotor

root = logging.getLogger()
root.setLevel(logging.INFO)

camera_vpos=0
camera_hpos=0
desired_face_xpos=320
desired_face_ypos=240
pixel_slop=10
servo_step=0.05

def acquire_camera_image(robot):
    #returns opencv camera image, in grayscale? TODO
    return None

def move_camera_to(robot,pitch,yaw):
    robot["camera_pitch_servo"].setpos_fraction(pitch)
    robot["camera_yaw_servo"].setpos_fraction(yaw)

def clip_servo_fraction(x):
    if x>1.0:
        return 1.0
    if x<-1.0:
        return -1.0
    return x

def move_servo_to_target(x,delta,slop,servo_step):
    if delta<-slop:
        x+=servo_step
    elif delta>slop:
        x-=servo_step
    return clip_servo_fraction(x)


#This is where our main function begins
if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize opencv
    face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
    eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    robot["camera_pitch_servo"].setpos_fraction(camera_vpos)
    robot["camera_yaw_servo"].setpos_fraction(camera_hpos)
    try:
        while True:
            #acquire camera image
            img=acquire_camera_image(robot)
            #identify where a face is
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            if len(faces)>0:
                (x,y,w,h)=faces[0]
                target_x=x+w/2
                target_y=y+w/2
                deltax=target_x-desired_face_xpos
                deltay=target_y-desired_face_ypos
                logging.info("Face Delta ({},{})".format(deltax,deltay))
                camera_hpos=move_servo_to_target(camera_hpos,deltax,pixel_slop,servo_step)
                camera_vpos=move_servo_to_target(camera_vpos,deltay,pixel_slop,servo_step)
                move_camera_to(robot,camera_hpos,camera_vpos)
            else:
                logging.info("No faces found")
            os.sleep(2)
                
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)
        print("all done")




