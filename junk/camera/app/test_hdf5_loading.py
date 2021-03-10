import h5py
import numpy as np
import cv2 as cv

hdf5_file=h5py.File("face_frames.h5","r")
xpos_array=hdf5_file["xpos_array"][...]
ypos_array=hdf5_file["ypos_array"][...]
xcontrol_array=hdf5_file["xcontrol_array"][...]
ycontrol_array=hdf5_file["ycontrol_array"][...]
frames=hdf5_file["frames"][...]


##cv.imshow('image',frames[0])
#cv.waitKey(0)
#cv.destroyAllWindows()

gray_frames=[]
for i in range(len(frames)):
    gray_frames.append(cv.cvtColor(frames[i].astype(np.uint8),cv.COLOR_BGR2GRAY))
# params for ShiTomasi corner detection
#feature_params = dict( maxCorners = 100,
#                       qualityLevel = 0.3,
#                       minDistance = 7,
#                       blockSize = 7 )
# Parameters for lucas kanade optical flow
#lk_params = dict( winSize  = (15,15),
#                  maxLevel = 2,
#                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

#p0 = cv.goodFeaturesToTrack(gray_frames[0], mask = None, **feature_params)
xflow=[0]
yflow=[0]
for i in range(len(frames)-1):
#p1,st,err=cv.calcOpticalFlowPyrLK(gray_frames[i],gray_frames[i+1],
    flow = cv.calcOpticalFlowFarneback(gray_frames[i],gray_frames[i+1], None, 0.5, 3, 15, 3, 5, 1.2, 0)
    xsum=0
    ysum=0
    for j in range(len(flow)):
        for k in range(len(flow[0])):
            xsum+=flow[j][k][0]
            ysum+=flow[j][k][1]
    xflow.append(xsum/(640*480))
    yflow.append(ysum/(640*480))

#print("flow {} {}".format(xsum,ysum))
for i in range(len(xpos_array)):
    print("{} {} {} {} {} {}".format(xpos_array[i],ypos_array[i],xcontrol_array[i],ycontrol_array[i],xflow[i],yflow[i]))


