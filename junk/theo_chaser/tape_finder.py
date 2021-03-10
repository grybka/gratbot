import cv2 as cv
import numpy as np
import logging
import math
from scipy.stats import linregress


# define range of blue color in HSV
#all lights on (100,252, 170)
#window closed (100,255,134)
#lights off (120 or 0, 255, 3)
#windo only (99, 245 , 148)
#old blue tape
#lower_target = np.array([90,200,100])
#upper_target = np.array([110,255,250])
#new blue tape
lower_target = np.array([90,75,100])
upper_target = np.array([110,200,250])


def process_img_to_find_tape(img):
    hsv=cv.cvtColor(img,cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_target,upper_target)
    #a form of blurrig

    xs=[]
    ys=[]
    #bottom_start=240
    bottom_start=340
    for y in range(bottom_start,480):
        mysum=0
        mycount=0
        for x in range(0,640):
            if mask[y][x]>0:
                mysum+=x
                mycount+=1
        if mycount>4:
            xpos=int(round(mysum/mycount))
            #img[y][xpos][0]=255
            #img[y][xpos][1]=0
            #img[y][xpos][2]=0
            xs.append(mysum/mycount)
            ys.append(y)
    fit=None
    if len(ys)>4:
        fit=linregress(ys,xs)
        m=fit.slope
        b=fit.intercept
    #    cv.line(img, (int(b+m*240), 240), (int(b+m*480), 480),(255,255,255), 3, cv.LINE_AA)
    return fit


def process_img_to_highlight_tape(img):
    hsv=cv.cvtColor(img,cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_target,upper_target)
    #a form of blurrig

    xs=[]
    ys=[]
    #bottom_start=240
    bottom_start=340
    for y in range(bottom_start,480):
        mysum=0
        mycount=0
        for x in range(0,640):
            if mask[y][x]>0:
                img[y][x][0]=0
                img[y][x][1]=0
                img[y][x][2]=255
                mysum+=x
                mycount+=1
        if mycount>4:
            xpos=int(round(mysum/mycount))
            img[y][xpos][0]=255
            img[y][xpos][1]=0
            img[y][xpos][2]=0
            xs.append(mysum/mycount)
            ys.append(y)
    fit=None
    if len(ys)>4:
        fit=linregress(ys,xs)
        m=fit.slope
        b=fit.intercept
    #    cv.line(img, (int(b+m*240), 240), (int(b+m*480), 480),(255,255,255), 3, cv.LINE_AA)
    return fit
