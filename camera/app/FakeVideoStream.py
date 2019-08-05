import cv2
import logging
import time
from GratbotObjectFinder import GratbotObjectFinder

class FakeVideoStream:
    def __init__(self,thefile):
        self.frame=cv2.imread(thefile)

    def read(self):
        return self.frame.copy()

notvideo=FakeVideoStream("personanddog.jpeg")
origframe=notvideo.read()
#print("shape {}".format(origframe.shape))
imagefinder=GratbotObjectFinder(notvideo)
cv2.namedWindow("preview1")
cv2.namedWindow("preview2")

try:
    starttime=time.time()
    while True:
        if time.time()-starttime>5:
            print("fps {}".format(imagefinder.get_fps()))
            starttime=time.time()
        origframe=notvideo.read()
        capframe=imagefinder.get_processed_frame()
        cv2.imshow("preview1",origframe)
        if len(capframe)>0:
            cv2.imshow("preview2",capframe)
        key=cv2.waitKey(30)
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
