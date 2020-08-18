import sys
import numpy as np
import cv2 as cv
import logging
import time
#import cvlib
#from cvlib.object_detection import draw_bbox

sys.path.append('../gratbot_client')
from GratbotComms import GratbotComms
from GratbotClient import GratbotClient
from rolly_behaviors import DisplayCamera
#from rolly_manualbehavior import XBoxControl
#from rolly_chase import RollyChase

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)
    #level=logging.DEBUG)

# connect to bot controls
logging.info("Connecting to Gratbot comms")
gratbot_comms = GratbotComms("10.0.0.4", 9999)
#go got to a good starting state
gratbot_comms.set_intention( [ "wheel_motor","stop", "SET" ], 1)

gratbot_comms.set_intention( ["wheel_turn_servo","position","SET" ], 0 )
gratbot_comms.set_intention( ["camera_pitch_servo","position","SET" ], 0 )
gratbot_comms.set_intention( ["camera_yaw_servo","position","SET" ], 0 )

# connect to camera
cv.namedWindow("preview")
cv.moveWindow("preview", 0, 0)

keep_going=True
on_behavior=DisplayCamera(gratbot_comms)
#on_behavior=XBoxControl(gratbot_comms)
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
finally:
    on_behavior.shut_down()
    logging.warning("telling motors to stop")
    gratbot_comms.set_intention( ["wheel_motor","stop","SET" ], 1)
    logging.warning("watiing for motors to stop")
    time.sleep(5)
    logging.warning("turning off comms ")
    gratbot_comms.stop()
logging.warning("all done ")
