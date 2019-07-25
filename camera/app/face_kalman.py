import numpy as np
import cv2 as cv
import time

import logging
#import filterpy
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise
from VisualTracker import VisualTracker
root = logging.getLogger()
root.setLevel(logging.INFO)

cv.namedWindow("preview")
vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
vc.set(3,320) #x res?
vc.set(4,240) #y res



#face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

if not vc.isOpened():
    print("could not open video")

tracker=VisualTracker('haarcascade_frontalface_default.xml')

#kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
#kfy = KalmanFilter(dim_x=2,dim_z=1) #ypos, yvel,      ymeas

# assign initial value for position velocity
#xpos,ypos,xvel,yvel=160.,120.,0.0,0.0
#kfx.x=np.array([xpos, xvel])
#kfy.x=np.array([ypos, yvel])
#transition matrix
#kfx.F=np.array([ [1.0,1.0],
#                [0.0,1.0] ])
#kfy.F=np.array([ [1.0,1.0],
#                [0.0,1.0] ])
##measurement function
#kfx.H=np.array([[1.0,0.0]])
#kfy.H=np.array([[1.0,0.0]])
##covariance matrix
#kfx.P *= 1000. #I guess I figure this is big 
#kfy.P *= 1000. #I guess I figure this is big 
##Fundamental maesurement noise
#kfx.R =np.array([[10.]])
#kfy.R =np.array([[10.]])
#kfx.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=10)
#kfy.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=10)



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
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        tracker.handle_next_frame(gray)
        tracker.highlight_frame(frame)
        logging.info("target seen in {} frames".format(np.sum(tracker.target_seen_array)))
    
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

