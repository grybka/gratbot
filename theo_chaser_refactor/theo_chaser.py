
import sys
import numpy as np
import threading
import cv2 as cv
import logging
import time
import os
import traceback
#import cvlib
#from cvlib.object_detection import draw_bbox

sys.path.append('../gratbot_client')
#from GratbotComms import GratbotComms
from GratbotCommsMk2 import GratbotCommsMk2
from GratbotClient import GratbotClient
from GratbotSensorFusion import GratbotSensorFusion

from behaviors.CalibrateMagsensor import CalibrateMagsensor
from behaviors.CalibrateMagsensor import CalibrateMagsensorPrintField
from behaviors.GratbotBehavior import GratbotBehavior_Wait
from behaviors.GratbotBehavior import GratbotBehavior_Series
from behaviors.GratbotBehavior import GratbotBehaviorStatus
from behaviors.GratbotBehavior import GratbotChoice
from behaviors.GratbotBehavior import GratbotDoUntil
from behaviors.CalibrateTurning import CalibrateTurnToAngle
from behaviors.CalibrateTurning import CalibrateTurnToVideo
from behaviors.CalibrateTurning import TurnRandomTime
from behaviors.CalibrateTurning import TurnRandomAndBack
from behaviors.CalibrateTurning import PrintTrackInfo
from behaviors.CalibrateFB import CalibrateFBToDistance
from behaviors.CalibrateFB import IsUltrasonic
from behaviors.CalibrateFB import FBRandomAmount
from behaviors.Automapper import TurnFixedAmount
from behaviors.Automapper import ForwardFixedAmount
from behaviors.Automapper import TurnToHeading
logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)

class DisplayLoop:
    def __init__(self,sensor_fusion):
        self.sensor_fusion=sensor_fusion
        self.keep_going=True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        self.primary_window=cv.namedWindow("vision")
        while self.keep_going:
            new_frames=self.sensor_fusion.get_user_display()
            for frm in new_frames:
                #if frm=="vision":
                if new_frames[frm] is not None:
                    cv.imshow(frm, new_frames[frm])
            #if new_frame is not None:
            #    cv.imshow("preview", new_frame)
            key = cv.waitKey(30)
        logging.warning("closing window ")
        cv.destroyAllWindows()

    def stop(self):
        self.keep_going=False
        self.thread.join()



# connect to bot controls
logging.info("Connecting to Gratbot comms")
gratbot_comms = GratbotCommsMk2("10.0.0.4", 9999)
gratbot_comms.set(["ultrasonic_sensor","update_frequency"],4)


sensor_fusion=GratbotSensorFusion()
sensor_fusion.load_config("sensor_fusion_config.yml")
display_loop=DisplayLoop(sensor_fusion)


#square_loop=GratbotBehavior_Series([GratbotBehavior_Wait(0.5),TurnFixedAmount(1.57),GratbotBehavior_Wait(0.5),ForwardFixedAmount(0.2)])
sensor_fusion.save_updates=True
#square_loop=GratbotBehavior_Series([TurnToHeading(0,0.05),ForwardFixedAmount(0.2),
#                                   TurnToHeading(np.pi/2,0.05),ForwardFixedAmount(0.2),
#                                   TurnToHeading(np.pi,0.05),ForwardFixedAmount(0.2),
#                                   TurnToHeading(3*np.pi/2,0.05),ForwardFixedAmount(0.2)])
#square_loop.should_loop=True
#on_behavior=square_loop
#turn_random_loop=GratbotBehavior_Series([TurnRandomTime(),GratbotBehavior_Wait(1.0)])
#turn_random_loop=GratbotBehavior_Series([TurnRandomAndBack(),GratbotBehavior_Wait(1.0)])
#turn_random_loop.should_loop=True
#on_behavior=turn_random_loop
#####forward backard
fb_forward=FBRandomAmount(0.3,0)
fb_backward=FBRandomAmount(0,-0.3)
fb_mid=FBRandomAmount(0.3,-0.3)
#fb_choice=GratbotChoice(IsUltrasonic(True,1.5),fb_backward,GratbotChoice(IsUltrasonic(False,0.5),fb_forward,fb_mid))
#fb_loop=GratbotBehavior_Series([GratbotBehavior_Wait(1.0),fb_choice])
goback=GratbotDoUntil(IsUltrasonic(False,1.5),GratbotBehavior_Series([GratbotBehavior_Wait(1.0),fb_backward]))
goforward=GratbotDoUntil(IsUltrasonic(True,0.5),GratbotBehavior_Series([GratbotBehavior_Wait(1.0),fb_forward]))
fb_loop=GratbotBehavior_Series([ goback,goforward])
fb_loop.should_loop=True
#on_behavior=fb_loop
#fb_loop=GratbotBehavior_Series([GratbotBehavior_Wait(1.0),GratbotChoice(IsUltrasonic(True,))])

#automapper_loop=GratbotBehavior_Series([GratbotBehavior_Wait(0.5),TurnFixedAmount(0.1)])
#automapper_loop.should_loop=True
#on_behavior=automapper_loop
#myloop=GratbotBehavior_Series([CalibrateMagsensorPrintField(),GratbotBehavior_Wait(0.5)])
myloop=GratbotBehavior_Series([PrintTrackInfo(),GratbotBehavior_Wait(0.5)])
myloop.should_loop=True
#on_behavior=CalibrateMagsensor()
on_behavior=myloop
#on_behavior=CalibrateTurnToAngle()
#on_behavior=CalibrateTurnToVideo()
#on_behavior=CalibrateFBToDistance()

try:
    #This is the behavior execution loop
    while True:
        time.sleep(0.03)
        #Get updates from sensors
        sensor_fusion.update(gratbot_comms)
        sensor_fusion.update_video()
        if on_behavior is not None:
            resp=on_behavior.act(gratbot_comms,sensor_fusion)
            if resp==GratbotBehaviorStatus.COMPLETED:
                print("Behavior Completed.  Halting")
                on_behavior=None

except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, saving config and exiting")
    sensor_fusion.save_config("sensor_fusion_config.yml")
except Exception as e:
    print("Exception: {}".format(e))
    exc_type, exc_obj, exc_tb = sys.exc_info()
    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    print(exc_type, fname, exc_tb.tb_lineno)
    traceback.print_exc(file=sys.stdout)
finally:
    display_loop.stop()
    sensor_fusion.stop()
