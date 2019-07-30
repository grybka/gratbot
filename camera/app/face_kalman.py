import numpy as np
import cv2 as cv
import time

import logging
#import filterpy
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise
from VisualTracker import VisualTracker
from NNServorControl import ControlsPredictor
root = logging.getLogger()
root.setLevel(logging.INFO)

max_servo_delta=10

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

def clamp_max(x,mx):
    if x>mx:
        return mx
    if x<-mx:
        return -mx
    return x



gratbot=GratbotClient("10.0.0.5",9999)
gratbot.connect()



cv.namedWindow("preview")
vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
#vc.set(3,320) #x res?
#vc.set(4,240) #y res

frame_width=vc.get(CAP_PROP_FRAME_WIDTH)
frame_height=vc.get(CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width,frame_height))

x_servo=ControlsPredictor()
y_servo=ControlsPredictor()

#face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

if not vc.isOpened():
    print("could not open video")

tracker=VisualTracker('haarcascade_frontalface_default.xml')

x_history=[]
y_history=[]
x_control_history=[]
y_control_history=[]
camera_vpos=300
camera_hpos=300



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
        rval,frame=vc.read()
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
                camera_hpos+=x_control
                camera_vpos+=y_control
                response=gratbot.send_message(["camera_pitch_servo","position_steps"],"SET",int(camera_vpos))
                response=gratbot.send_message(["camera_yaw_servo","position_steps"],"SET",int(camera_hpos))
            else:
                x_control=0
                y_control=0
            x_control_history.append(x_control)
            y_control_history.append(y_control)
            train_counter+=1
            if train_counter>=min_training_size:
                logging.info("Training")
                x_servo.train(x_history,x_control_history)
                y_servo.train(y_history,y_control_history)
                
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
        cv.imshow("preview",frame)
        key=cv.waitKey(10)
        step_size=10
        if key!=-1:
            logging.info("key pressed {}".format(key))
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
finally:        
    logging.warning("Releasing videocapture")
    vc.release()

