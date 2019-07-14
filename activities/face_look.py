import sys
import yaml
import time
import logging
import traceback
import picamera
import picamera.array
import numpy as np
import cv2 as cv
import os
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotMotor

root = logging.getLogger()
root.setLevel(logging.INFO)

camera = picamera.PiCamera()              #Camera initialization
xres=640
yres=480
camera.resolution = (xres, yres)
camera.framerate = 6

camera_vpos=0
camera_hpos=0
desired_face_xpos=320
desired_face_ypos=240
pixel_slop=10
servo_step=0.05

def acquire_camera_image(robot):
    #returns opencv camera image, in grayscale? TODO
    rawCapture=picamera.array.PiRGBArray(camera)
    camera.capture(rawCapture,format="bgr")
    return cv.cvtColor(rawCapture.array, cv.COLOR_BGR2GRAY)
    #return rawCapture.array

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

def highlight_face(facearray,img,num):
    for (x,y,w,h) in faces:
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
        #for (ex,ey,ew,eh) in eyes:
        #    cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        #cv.imgwrite("x.png",img)
        cv.imwrite("x_{}.png".format(num),img)

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
    loopnumber=0
    try:
        while True:
            #acquire camera image
            img=acquire_camera_image(robot)
            #identify where a face is
            faces = face_cascade.detectMultiScale(img, 1.3, 5)
            if len(faces)>0:
                highlight_face(faces,img,loopnumber)
                loopnumber=loopnumber+1
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
            time.sleep(2)
                
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)
        print("all done")




