import numpy as np
import cv2 as cv
import logging
import time
import threading
from xbox360controller import Xbox360Controller

from GratbotClient import GratbotClient
from GratbotVideoConnectionUV4L import GratbotVideoConnectionUV4L
from GratbotObjectFinder import GratbotObjectFinder
from GratbotComms import GratbotComms
from GratbotBehavior import GratbotBehaviorHunt


#connect to bot controls
gratbot_comms=GratbotComms("10.0.0.5",9999)



#------- controller callbacks -------
class controller_info:
    def __init__(self):
        self.enabled=True

cinfo=controller_info()

def dead_zoneify(point):
    vx=point[0]
    vy=point[1]
    if abs(vx)<0.5:
        vx=0
    if abs(vy)<0.5:
        vy=0
    return vx,vy
   

def on_axis_l_moved(axis):
    if cinfo.enabled:
        vx,vy=dead_zoneify( [axis.x,axis.y] )
        wheel_yaw_neutral=370
        wheel_yaw_spread=130
        toset=wheel_yaw_neutral+wheel_yaw_spread*vx
        gratbot_comms.set_intention( [ "wheel_turn_servo","position_steps", "SET" ], toset )
        gratbot_comms.set_intention( [ "wheel_motor","speed", "SET" ], 100*vy)

logging.info("logger test")
logging.warning("this is a warning")
def on_button_a_pressed(button):
    if cinfo.enabled:
        logging.info("Controller Disabled")
        cinfo.enabled=False
    else:
        logging.info("Controller Enabled")
        cinfo.enabled=True



#connect to controller
controller=Xbox360Controller(0, axis_threshold=0.2)
controller.axis_l.when_moved=on_axis_l_moved
controller.button_a.when_pressed = on_button_a_pressed

#---------------------------


#connect to camera
cv.namedWindow("preview")
cv.moveWindow("preview",0,0)
video=GratbotVideoConnectionUV4L("http://10.0.0.5:8080/stream/video.mjpeg")
frame_width=video.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frame_height=video.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width,frame_height))

#start the image finder 
imagefinder=GratbotObjectFinder(video)
cv.namedWindow("object_finder")
cv.moveWindow("object_finder",320,0)

#instatiate behaviors
hunting=GratbotBehaviorHunt(gratbot_comms,imagefinder)
#behaviour loop
def behavior_loop():
    while not behavior_thread_should_quit:
        if not cinfo.enabled:
            hunting.act()
        time.sleep(0.1)
            

behavior_thread_should_quit=False
behavior_thread=threading.Thread(target=behavior_loop)
behavior_thread.daemon=True
behavior_thread.start()


#Video Display loop here
try:
    cycle_counter=0
    while True:
        cycle_counter+=1
        myframe=video.read()
        myframe=cv.resize(myframe,None,fx=4,fy=4)
        cv.imshow("preview",myframe)
        objframe=imagefinder.get_processed_frame()
        if len(objframe)>0:
            objframe=cv.resize(objframe,None,fx=4,fy=4)
#cv.putText(objframe,"Dist: {0:.3g} cm".format(gratbot_comms.ultra),(320,200),cv.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0))
            cv.imshow("object_finder",objframe)
        key=cv.waitKey(10)
#if cycle_counter%30==0:
#print("object fps {}".format(imagefinder.get_fps()))
        #send commands (could be different thread)
        
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
    behavior_thread_should_quit=True
    hunting.save_state()
finally:        
    video.stop()
    controller.close()
    gratbot_comms.stop()
