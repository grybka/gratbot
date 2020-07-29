import sys
import numpy as np
import cv2 as cv
import logging
#import cvlib
#from cvlib.object_detection import draw_bbox

from GratbotVideoConnectionUV4L import GratbotVideoConnectionUV4L
sys.path.append('../gratbot_client')
from GratbotComms import GratbotComms
from GratbotClient import GratbotClient
from GratbotBehaviors import WaveAtFace
from GratbotBehaviors import HeadTrackFace
from GratbotBehaviors import MoveAndTrackObjects
from GratbotBehaviors import JustSaveObjectPos
from GratbotBehaviors import ShowColorHisto
from GratbotBehaviors import HighlightColor
from GratbotManualControls import XBoxControl

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)
#    level=logging.DEBUG)

# connect to bot controls
logging.info("Connecting to Gratbot comms")
gratbot_comms = GratbotComms("10.0.0.5", 9999)


# connect to camera
cv.namedWindow("preview")
cv.moveWindow("preview", 0, 0)
logging.info("Connecting to Gratbot video")
video = GratbotVideoConnectionUV4L("http://10.0.0.5:8080/stream/video.mjpeg")
frame_width = video.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frame_height = video.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width, frame_height))

forward_speed=0.0
#on_behavior= WaveAtFace(gratbot_comms)
#on_behavior = HeadTrackFace(gratbot_comms)
gratbot_comms.set_intention( ["camera_x","position","SET" ], 0 )
gratbot_comms.set_intention( ["camera_y","position","SET" ], -20 )
#gratbot_comms.set_intention( ["camera_y","position","SET" ], 0 )
#on_behavior = MoveAndTrackObjects(gratbot_comms)
on_behavior = XBoxControl(gratbot_comms)
#on_behavior = JustSaveObjectPos(gratbot_comms)
#on_behavior = ShowColorHisto(gratbot_comms)
#on_behavior = HighlightColor(gratbot_comms)
#on_behavior = None
keep_going=True

def shut_down():
    keep_going=False
    gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
    on_behavior.shut_down()
    logging.warning("stopping video")
    video.stop()
    #controller.close()
    logging.warning("turning off comms ")
    gratbot_comms.stop()

# Video Display loop here
try:
    cycle_counter = 0
    while keep_going:
        cycle_counter += 1
        myframe, mytime = video.read()
        #myframe = cv.resize(myframe, None, fx=4, fy=4)
        #objframe = imagefinder.get_processed_frame()
#        objframe = []
#        if len(objframe) > 0:
#            objframe = cv.resize(objframe, None, fx=4, fy=4)
#cv.putText(objframe,"Dist: {0:.3g} cm".format(gratbot_comms.ultra),(320,200),cv.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0))
#            cv.imshow("object_finder", objframe)
        key = cv.waitKey(10)
        if on_behavior is not None:
            new_frame=on_behavior.act(myframe)
            if new_frame is not None:
                cv.imshow("preview", new_frame)
        else:
            cv.imshow("preview", myframe)
        if key!=-1:
            print("{} pressed".format(key))
            if key==97: #a Key
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], 0)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], 0)
            if key==44: #w Key in dvorak
                forward_speed=np.clip(forward_speed+0.1,-1,1)
                print("Speed {}".format(forward_speed))
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], forward_speed)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], forward_speed)
            if key==111: #s Key idvorake
                forward_speed=np.clip(forward_speed-0.1,-1,1)
                print("Speed {}".format(forward_speed))
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], forward_speed)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], forward_speed)
            if key==39: #q Key idvorake
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
                forward_speed=0
                shut_down()
# if cycle_counter%30==0:
#print("object fps {}".format(imagefinder.get_fps()))
        # send commands (could be different thread)

except KeyboardInterrupt:
    gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
    logging.warning("Keyboard Exception Program Ended, exiting")
finally:
    on_behavior.shut_down()
    logging.warning("stopping video")
    video.stop()
    #controller.close()
    logging.warning("turning off comms ")
    gratbot_comms.stop()
logging.warning("all done ")
