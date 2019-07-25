import numpy as np
import cv2 as cv
import time
import logging

root = logging.getLogger()
root.setLevel(logging.INFO)


cv.namedWindow("preview")
vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
vc.set(3,320) #x res?
vc.set(4,240) #y res

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

if not vc.isOpened():
    print("could not open video")

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
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        cv.imshow("preview",frame)
        key=cv.waitKey(30)
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
finally:        
    logging.warning("Releasing videocapture")
    vc.release()
