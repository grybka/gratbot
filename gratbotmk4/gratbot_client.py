#client to connect to server
import sys,os,traceback,time,threading
sys.path.append('gyrii')
sys.path.append('network')
import logging
import cv2 as cv
import argparse
from network.JSONBackAndForthServer import JSONBackAndForth
from MessageBroker import MessageBroker
from gyrii.Gyrus import GyrusList,VideoDisplay
from gyrii.SocketGyrusLink import SocketGyrusLink
from gyrii.HeadTrackerGyrus import HeadTrackerGyrus
from gyrii.ReplayGyrus import ReplayGyrus
from gyrii.MessageLoggerGyrus import MessageLoggerGyrus
from gyrii.CameraDisplayGyrus import CameraDisplayGyrus
#from gyrii.TrackerGyrus import TrackerGyrus
from gyrii.TrackerGyrusNoCV import TrackerGyrusNoCV
from gyrii.BehaviorGyrus import BehaviorGyrus
#from gyrii.MotionGyrus import MotionGyrus
from gyrii.XboxControllerGyrus import XboxControllerGyrus
#from gyrii.behaviors.TextCommandBehavior import TextCommandBehavior
from gyrii.behaviors.CalibrateMotionBehavior import CalibrateMotionBehavior,ExerciseTurns,CalibrateMotionBehavior_WithTracking_Turns,CalibrateMotionBehaviorFB,CalibrateMotionBehavior_WithTracking_FB
from gyrii.behaviors.ChaseBehavior import TrackIfSeen
from gyrii.ClockGyrus import ClockGyrus

argparser = argparse.ArgumentParser(description='Gratbot client')
argparser.add_argument('--sim', action='store_true',help="Run simulated data instead of real connection")
args=argparser.parse_args()

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)

class DisplayLoop(VideoDisplay):
    def __init__(self):
        self.window_images={}
        self.open_windows=[]
        self.frame_lock=threading.Lock()

    def update_image(self,windowname,image):
        self.frame_lock.acquire()
        self.window_images[windowname]=image
        self.frame_lock.release()

    def one_loop(self):
        self.frame_lock.acquire()
        for key in self.window_images:
            if key not in self.open_windows:
                cv.namedWindow(key)
                self.open_windows.append(key)
            cv.imshow(key, self.window_images[key])
        self.frame_lock.release()
        key = cv.waitKey(30)

    def __del__(self):
        cv.destroyAllWindows()
display_loop=DisplayLoop()


#This stores the message passing
broker=MessageBroker()


logging.debug("Creating Gyrus List")
gyrii=GyrusList()
if args.sim:
    logging.debug("Starting Simulation")
    gyrii.append(ReplayGyrus(broker,"config/sim_input.txt"))
    #gyrii.append(ReplayGyrus(broker,"config/sim_input_ball_rolls.txt",slomo=1))
else:
    logging.debug("Starting Server")
    test_port=23033
    server_address="10.0.0.4"
    network_client=JSONBackAndForth()
    network_client.start_client(server_address,test_port)
    gyrii.append(SocketGyrusLink(broker,network_client.input_queue,network_client.output_queue,keys=["motor_command","servo_command","behavior_request"]))
gyrii.append(MessageLoggerGyrus(broker,keys=["rotation_vector","detections","motor_command","motor_response","tracks","servo_response"]))
gyrii.append(CameraDisplayGyrus(broker,display_loop))
#gyrii.append(BehaviorGyrus(broker,CalibrateMotionBehavior()))
#gyrii.append(BehaviorGyrus(broker,ExerciseTurns()))
#gyrii.append(BehaviorGyrus(broker,TrackIfSeen()))
#gyrii.append(BehaviorGyrus(broker,None))
#gyrii.append(BehaviorGyrus(broker,CalibrateMotionBehavior_WithTracking_Turns(["sports ball","orange"])))
#gyrii.append(BehaviorGyrus(broker,CalibrateMotionBehavior_WithTracking_FB(["sports ball","orange"])))
#gyrii.append(HeadTrackerGyrus(broker))
#gyrii.append(TrackerGyrusNoCV(broker))
gyrii.append(XboxControllerGyrus(broker))
#gyrii.append(MotionGyrus(broker))
gyrii.append(ClockGyrus(broker))

def main():
    try:
        config_filename="config/client_gyrus_config.yaml"
        logging.debug("configuring and starting gyrii")
        gyrii.config_and_start(config_filename)
        logging.debug("gyrii started")
        while True:
            display_loop.one_loop()

    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended")
    except Exception as e:
        print("Exception: {}".format(e))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        traceback.print_exc(file=sys.stdout)
    finally:
        gyrii.quit_and_join()

main()
