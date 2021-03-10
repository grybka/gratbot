import numpy as np
import cv2 as cv
import time
import queue, threading, time
import logging
import socket
import json
#import filterpy
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise
from VisualTracker import VisualTracker
from NNServoControl import ControlsPredictor
from GratbotClient import GratbotClient
import h5py
root = logging.getLogger()
root.setLevel(logging.INFO)

max_servo_delta=5

def clamp_max(x,mx):
    if x>mx:
        return mx
    if x<-mx:
        return -mx
    return x



gratbot=GratbotClient("10.0.0.5",9999)
gratbot.connect()

# bufferless VideoCapture
class VideoCapture:
  def __init__(self, name):
    self.cap = cv.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()


cv.namedWindow("preview")
#vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
vc=VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")

img_array=[]
xpos_array=[]
ypos_array=[]
xcontrol_array=[]
ycontrol_array=[]
max_array_size=10
#vc.set(3,320) #x res?
#vc.set(4,240) #y res

frame_width=vc.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frame_height=vc.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width,frame_height))

x_servo=ControlsPredictor()
y_servo=ControlsPredictor()

#pretrain the controls predictior
input_arrays=[]
control_signals=[]
for i in range(200,500,5):
    input_arrays.append(float(i))
    control_signals.append(-5)
for i in range(500,200,-5):
    input_arrays.append(float(i))
    control_signals.append(5)
x_servo.train(input_arrays,control_signals)
y_servo.train(input_arrays,control_signals)



#face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

#if not vc.isOpened():
#    print("could not open video")

tracker=VisualTracker('haarcascade_frontalface_default.xml')

x_history=[]
y_history=[]
x_control_history=[]
y_control_history=[]
camera_vpos=420
camera_hpos=370
response=gratbot.send_message(["camera_pitch_servo","position_steps"],"SET",int(camera_vpos))
response=gratbot.send_message(["camera_yaw_servo","position_steps"],"SET",int(camera_hpos))
#print("attempting to {}".format(vc.set(cv.CAP_PROP_BUFFERSIZE, 0)))

min_training_size=10
try:
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

#rval,frame=vc.read()
        frame=vc.read()
#print("frame type {}".format(type(frame[0][0][0])))
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        tracker.handle_next_frame(gray)
        tracker.highlight_frame(frame)
        if tracker.test_existance():
            xpos=tracker.kfx.x[0]-frame_width/2
            ypos=tracker.kfy.x[0]-frame_height/2
            x_history.append(xpos)
            y_history.append(ypos)
            if len(x_history)>2:
                x_control=x_servo.predict(x_history,xpos)
                y_control=y_servo.predict(y_history,xpos)
                x_control=int(clamp_max(x_control,max_servo_delta))
                y_control=int(clamp_max(x_control,max_servo_delta))
                logging.info("xpos ypos {} {}".format(xpos,ypos))
                logging.info("x control y control {} {}".format(x_control,y_control))
                camera_hpos+=x_control
                camera_vpos+=y_control
                response=gratbot.send_message(["camera_pitch_servo","position_steps"],"SET",int(camera_vpos))
                response=gratbot.send_message(["camera_yaw_servo","position_steps"],"SET",int(camera_hpos))
            else:
                x_control=0
                y_control=0
            if len(xpos_array)<max_array_size:
                xcontrol_array.append(x_control)
                ycontrol_array.append(y_control)
                xpos_array.append(camera_hpos)
                ypos_array.append(camera_vpos)
                img_array.append(frame)


            x_control_history.append(x_control)
            y_control_history.append(y_control)
            train_counter+=1
            if train_counter>=min_training_size:
                logging.info("Training")
                x_servo.train(x_history,x_control_history)
                y_servo.train(y_history,y_control_history)
                logging.info("Parameters {}".format(x_servo.parameters()))
                
        else:
            #erase history of tracks
            train_counter=0
            x_history=[]
            y_history=[]
            x_control_history=[]
            y_control_history=[]



#logging.info("target seen in {} frames".format(np.sum(tracker.target_seen_array)))
    
        #----do face tracking here
#        dothing=False
#        dothing=True
#        kfx.predict()
#        kfy.predict()
#        if dothing:
#            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
#            for (x,y,w,h) in faces:
#                cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
#            if len(faces)>0:
#                    (x,y,w,h)=faces[0]
#                    x_center=x+w/2
#                    y_center=y+w/2
#                    kfx.update([x_center])
#                    kfy.update(np.array([y_center]))
#            cv.rectangle(frame,(int(kfx.x[0]-10),int(kfy.x[0])-10),(int(kfx.x[0])+10,int(kfy.x[0])+10),(255,0,0),2)
#        logging.info("unceratienty x {}".format(kfx.S))
#        logging.info("unceratienty y {}".format(kfy.S))
        
            
#target_x=x+w/2
#target_y=y+w/2
#logging.info("Target Pos ({},{})".format(target_x,target_y))
#deltax=target_x-desired_face_xpos
#deltay=target_y-desired_face_ypos
#logging.info("Face Delta ({},{})".format(deltax,deltay))
        #--------
        if onframe%2==0:
            cv.imshow("preview",frame)
            key=cv.waitKey(10)
            if key!=-1:
                logging.info("key pressed {}".format(key))
        step_size=10
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
    hdf5_file=h5py.File("face_frames.h5",mode="w")
    hdf5_file.create_dataset("xpos_array",(len(xpos_array),),np.int32)
    hdf5_file["xpos_array"][...]=xpos_array
    hdf5_file.create_dataset("ypos_array",(len(ypos_array),),np.int32)
    hdf5_file["ypos_array"][...]=ypos_array
    hdf5_file.create_dataset("xcontrol_array",(len(xcontrol_array),),np.int32)
    hdf5_file["xcontrol_array"][...]=xcontrol_array
    hdf5_file.create_dataset("ycontrol_array",(len(ycontrol_array),),np.int32)
    hdf5_file["ycontrol_array"][...]=ycontrol_array
    imgshape=(len(img_array),480,640,3)
    hdf5_file.create_dataset("frames",imgshape,np.uint8)
    hdf5_file["frames"][...]=img_array
    hdf5_file.close()

finally:        
    logging.warning("Releasing videocapture")
    vc.cap.release()

