import numpy as np
import cv2 as cv

cv.namedWindow("preview")
#vc=cv.VideoCapture("rtsp:10.0.0.5???")
vc=cv.VideoCapture("http://10.0.0.4:8080/stream/video.mjpeg")
#vc.set(cv.CV_CAP_PROP_BUFFERSIZE,3)
#vc=cv.VideoCapture("rtsp://10.0.0.4:8080/out.h264")
#vc.set(3,320) #x res?
#vc.set(4,240) #y res

if vc.isOpened():
    rval,frame=vc.read()
else:
    rval=False

while rval:
    rval,frame=vc.read()
    cv.imshow("preview",frame)
    key=cv.waitKey(1)
    if key==27:
        break
