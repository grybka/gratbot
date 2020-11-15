
import sys
import numpy as np
import cv2 as cv
import logging
import time
import os
import traceback
#import cvlib
#from cvlib.object_detection import draw_bbox

sys.path.append('../gratbot_client')
from GratbotComms import GratbotComms
from GratbotClient import GratbotClient
from theo_chaser_manualbehavior import XBoxControl
from theo_chaser_chase import Theo_Chaser_Chase

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)
    #level=logging.DEBUG)

# connect to bot controls
logging.info("Connecting to Gratbot comms")
gratbot_comms = GratbotComms("10.0.0.4", 9999)

# connect to camera
cv.namedWindow("preview")
cv.moveWindow("preview", 0, 0)

keep_going=True
#on_behavior=DisplayCamera(gratbot_comms)
#on_behavior=XBoxControl(gratbot_comms)
on_behavior=Theo_Chaser_Chase(gratbot_comms)
#on_behavior=RollyChase(gratbot_comms)

def shut_down():
    keep_going=False
    print("telling behavior to stop")
    on_behavior.shut_down()
    #controller.close()
    logging.warning("turning off comms ")
    gratbot_comms.stop()
    logging.warning("closing window ")

    cv.destroyAllWindows()

# Video Display loop here
video_state="start"
try:
    cycle_counter = 0
    while keep_going:
        cycle_counter += 1
        key = cv.waitKey(10)
        if on_behavior is not None:
            new_frame=on_behavior.act()
            if new_frame is not None:
                cv.imshow("preview", new_frame)
        if key!=-1:
            print("{} pressed".format(key))
            if key==97: #a Key
                pass
            if key==44: #w Key in dvorak
                pass
            if key==111: #s Key idvorake
                pass
            if key==39: #q Key idvorake
                pass
                print("calling shut down")
                shut_down()
                keep_going=False

except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
except Exception as e:
    print("Exception: {}".format(e))
    exc_type, exc_obj, exc_tb = sys.exc_info()
    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    print(exc_type, fname, exc_tb.tb_lineno)
    traceback.print_exc(file=sys.stdout)
finally:
    on_behavior.shut_down()
    logging.warning("telling motors to stop")
    gratbot_comms.set_intention(["drive", "translate", "SET"], [0, 0, 0])
    time.sleep(5)
    logging.warning("turning off comms ")
    gratbot_comms.stop()
logging.warning("all done ")
