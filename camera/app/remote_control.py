import numpy as np
import cv2 as cv
import time
import logging
import socket
import sys
import yaml
import json

root = logging.getLogger()
root.setLevel(logging.INFO)
desired_face_xpos=160
desired_face_ypos=120
pixel_slop=10
max_servo_step=0.20
servo_ratio=0.005


class GratbotClient:
    def __init__(self,host,port):
        self.host=host
        self.port=port
        self.sock=None

    def connect(self):
        #connect to server
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        return self.sock.connect((self.host,self.port))
    
    def disconnect(self):
        if not (self.sock is None):
            self.sock.close()
            self.sock=None

    def send_message(self,address,command,arguments):
        message={"address": address,"command": command,"arguments": arguments}
        logging.info("sending {}".format(json.dumps(message)))
        self.sock.sendall((json.dumps(message)+"\n").encode())
        data=self.sock.recv(1024)
#logging.info("received: {}".format(data))
        while len(data)==0 or data[-1]!=10:
            if len(data)>0:
                logging.info("received: {}".format(data))
                logging.info("last elem |{}|".format(data[-1]))
            data+=self.sock.recv(1024)
        logging.info("received: {}".format(data))
        return json.loads(data)
 
    def __del__(self):
        self.disconnect()

gratbot=GratbotClient("10.0.0.5",9999)
gratbot.connect()

cv.namedWindow("preview")
vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
vc.set(3,320) #x res?
vc.set(4,240) #y res

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

if not vc.isOpened():
    print("could not open video")


def move_camera_to(pitch,yaw):
    response=gratbot.send_message(["camera_pitch_servo","position"],"SET",float(pitch))
    logging.info("I recevied {}".format(response))
    if "error" in response:
        raise Exception("Error sending command: "+response["error"])
    response=gratbot.send_message(["camera_yaw_servo","position"],"SET",float(yaw))
    logging.info("I recevied {}".format(response))
    if "error" in response:
        raise Exception("Error sending command: "+response["error"])

def clip_servo_fraction(x):
    if x>1.0:
        return 1.0
    if x<-1.0:
        return -1.0
    return x

def move_servo_to_target(x,delta,slop,max_servo_step):
    if delta<-slop:
        x+=min(abs(delta)*servo_ratio,max_servo_step)
    elif delta>slop:
        x-=min(abs(delta)*servo_ratio,max_servo_step)
    return clip_servo_fraction(x)

camera_vpos=300
camera_hpos=300

try:
#sock.connect((HOST,PORT))
    onframe=0
    lasttime=time.time()
    while True:
        onframe=onframe+1
        if onframe%60==0:
            thistime=time.time()
            timedelta=thistime-lasttime
            fps=60/timedelta
            logging.info("FPS: {}".format(fps))
            lasttime=thistime
        rval,frame=vc.read()
    
        #----do face tracking here
        dothing=False
        dothing=True
        if dothing:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x,y,w,h) in faces:
                cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            if len(faces)>0:
                    (x,y,w,h)=faces[0]
                    target_x=x+w/2
                    target_y=y+w/2
                    logging.info("Target Pos ({},{})".format(target_x,target_y))
                    deltax=target_x-desired_face_xpos
                    deltay=target_y-desired_face_ypos
                    logging.info("Face Delta ({},{})".format(deltax,deltay))
        #--------
        cv.imshow("preview",frame)
        key=cv.waitKey(10)
        step_size=10
        if key!=-1:
            logging.info("key pressed {}".format(key))
        #420 is about neutral in pitch
        #340 is about minimal
        #500 is max
        #270 is far right
        #370 is neutral
        #510 is max
            
        if key==82: #Up
            camera_vpos+=step_size
            print("Pitch {}".format(camera_vpos))
            response=gratbot.send_message(["camera_pitch_servo","position_steps"],"SET",int(camera_vpos))
        if key==84: #Down
            camera_vpos-=step_size
            print("Pitch {}".format(camera_vpos))
            response=gratbot.send_message(["camera_pitch_servo","position_steps"],"SET",int(camera_vpos))
        if key==81: #Left
            camera_hpos+=step_size
            print("Yaw {}".format(camera_hpos))
            response=gratbot.send_message(["camera_yaw_servo","position_steps"],"SET",int(camera_hpos))
        if key==83: #Left
            camera_hpos-=step_size
            print("Yaw {}".format(camera_hpos))
            response=gratbot.send_message(["camera_yaw_servo","position_steps"],"SET",int(camera_hpos))

except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
finally:        
    logging.warning("Releasing videocapture")
    vc.release()
    gratbot.disconnect()
