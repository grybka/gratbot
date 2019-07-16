import numpy as np
import cv2 as cv

cv.namedWindow("preview")
#vc=cv.VideoCapture("rtsp:10.0.0.5???")
#vc=cv.VideoCapture("http://10.0.0.5/videostream.cgi?.mjpg")
vc=cv.VideoCapture("http://10.0.0.5:8080/stream/video.mjpeg")
vc.set(3,320) #x res?
vc.set(4,240) #y res

if vc.isOpened():
    rval,frame=vc.read()
else:
    rval=False

while rval:
    cv.imshow("preview",frame)
    rval,frame=vc.read()
    key=cv.waitKey(33)
    if key==27:
        break
