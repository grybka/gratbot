#Gratbot server
import sys,os,traceback,time
sys.path.append('gyrii')
sys.path.append('network')
import logging
#from network.JSONBackAndForthServer2 import JSONBackAndForth2
from network.JSONBackAndForthServerPickle import JSONBackAndForth2
from MessageBroker import MessageBroker
from gyrii.Gyrus import GyrusList
#from gyrii.SocketGyrusLink import SocketGyrusLink
from gyrii.SocketGyrusLinkPickle import SocketGyrusLink
from gyrii.MotorGyrus import MotorGyrus
from gyrii.ServoGyrus import ServoGyrus
from gyrii.ServoGyrusVelocity import ServoGyrusVelocity
#from gyrii.HeadTrackerGyrus import HeadTrackerGyrus,FollowerGyrus
#from gyrii.HeadTrackerGyrus import TurnTrackerGyrus
from gyrii.NeckGazeGyrus import  NeckPointingErrorCorrectionGyrus,PointingErrorGyrus,BodyPointingErrorCorrectionGyrus
from gyrii.MessageLoggerGyrus import MessageLoggerGyrus
from gyrii.TailGyrus import TailGyrus
#from gyrii.TrackerGyrus import TrackerGyrus
#from gyrii.TrackerGyrusNoCV import TrackerGyrusNoCV
#from gyrii.TrackerGyrus2 import TrackerGyrus
from gyrii.TrackerGyrus3 import TrackerGyrus
from gyrii.FastBadTrackerGyrus import FastBadTrackerGyrus
from gyrii.MicrophoneGyrus2 import MicrophoneGyrus
from gyrii.BehaviorGyrus import BehaviorGyrus
from gyrii.behaviors.Behavior import Announce
from gyrii.behaviors.CalibrateMotionBehavior import ServoVelUpAndDown,ServoUpAndDown,calibrate_neck_motion,calibrate_turn_motion,calibrate_turn_motion_motors_only

from gyrii.behaviors.FollowBehavior import find_and_follow, tail_test, look_around
from gyrii.SoundRecordGyrus import SoundRecordGyrus
#from gyrii.behaviors.ChaseBehavior import TrackIfSeen
from gyrii.ClockGyrus import ClockGyrus
from gyrii.LEDDisplayGyrus import LEDDisplayGyrus
#from SpeechDetectorGyrus import SpeechDetectorGyrus
#from OakDGyrus3 import OakDGyrus #generic objects
#from OakDGyrus4 import OakDGyrus #faces
#from OakDGyrusYolo import OakDGyrus #faces
#from OakDGyrusPeople import OakDGyrusPeople
from OakDGyrus5 import OakDGyrus

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)


#This stores the message passing
broker=MessageBroker()

#Open up a server to talk
logging.debug("Starting Server")
test_port=23033
network_server=JSONBackAndForth2(maxsize=10)
network_server.start_server(test_port)

logging.debug("Creating Gyrus List")
gyrii=GyrusList()
#gyrii.append(SocketGyrusLink(broker,network_server.input_queue,network_server.output_queue,keys=["rotation_vector","image","detections","motor_response","tracks","servo_response","logged_note","microphone_data"])) #TODO define keys here
gyrii.append(SocketGyrusLink(broker,network_server.input_queue,network_server.output_queue,keys=["rotation_vector","image","motor_response","tracks","servo_response","logged_note","microphone_data"])) #TODO define keys here
#gyrii.append(SocketGyrusLink(broker,network_server.input_queue,network_server.output_queue,keys=["rotation_vector","image","detections","motor_response","servo_response","logged_note","microphone_data"])) #TODO define keys here
#### Hardware interface #####

gyrii.append(ClockGyrus(broker))
gyrii.append(OakDGyrus(broker))
#gyrii.append(OakDGyrusPeople(broker))
gyrii.append(MotorGyrus(broker))
#gyrii.append(ServoGyrus(broker))
gyrii.append(ServoGyrusVelocity(broker))
#gyrii.append(LEDDisplayGyrus(broker))
#gyrii.append(MicrophoneGyrus(broker))


### Processing ####
#For Yolo
gyrii.append(TrackerGyrus(broker,confidence_trigger=0.3,detection_name="detections"))

#For Faces
#gyrii.append(TrackerGyrus(broker,confidence_trigger=0.9,detection_name="face_detections"))

#gyrii.append(TrackerGyrus(broker,confidence_trigger=0.7,detection_name="person_detections"))
#gyrii.append(FastBadTrackerGyrus(broker,confidence_trigger=0.2))
#gyrii.append(BehaviorGyrus(broker,ServoVelUpAndDown))

#gyrii.append(PointingErrorGyrus(broker,do_distance_corrections=True)) #figure out where I ought to point
gyrii.append(NeckPointingErrorCorrectionGyrus(broker,enabled=True)) #send servo commands to correct error
#gyrii.append(BodyPointingErrorCorrectionGyrus(broker,enabled=True)) #send motors commands to correct error

#gyrii.append(NeckGazeGyrus(broker))

##### Logging #####

#gyrii.append(MessageLoggerGyrus(broker,keys=["image","tracks"]))
#gyrii.append(MessageLoggerGyrus(broker,keys=["detections","tracks"]))
gyrii.append(MessageLoggerGyrus(broker,keys=["logged_note"]))
#gyrii.append(MessageLoggerGyrus(broker,keys=["rotation_vector","motor_command","motor_response","servo_command","servo_response","detections","tracks","logged_note","detections","image"]))
#gyrii.append(MessageLoggerGyrus(broker,keys=["rotation_vector","motor_command","motor_response","servo_command","servo_response","detections","tracks","logged_note","detections"]))
#gyrii.append(TrackerGyrusNoCV(broker,include_subimages=True))
#gyrii.append(HeadTrackerGyrus(broker))
#gyrii.append(FollowerGyrus(broker,only_turn=True))
#gyrii.append(BehaviorGyrus(broker,look_around()))
#gyrii.append(BehaviorGyrus(broker,find_and_follow(["face","person"])))
#gyrii.append(BehaviorGyrus(broker,tail_test()))
#gyrii.append(BehaviorGyrus(broker,Announce("announcement")))
#gyrii.append(BehaviorGyrus(broker,calibrate_neck_motion()))
#gyrii.append(TurnTrackerGyrus(broker))
#gyrii.append(BehaviorGyrus(broker,CalibrateMotionBehavior()))
#gyrii.append(BehaviorGyrus(broker,ServoUpAndDown()))
#gyrii.append(BehaviorGyrus(broker,calibrate_neck_motion()))
#gyrii.append(BehaviorGyrus(broker,calibrate_turn_motion_motors_only()))
#gyrii.append(BehaviorGyrus(broker,TrackIfSeen()))
#gyrii.append(BehaviorGyrus(broker,None))
#gyrii.append(TailGyrus(broker))
#gyrii.append(SoundRecordGyrus(broker))
#gyrii.append(SpeechDetectorGyrus(broker))

def main():
    try:
        #load gyrus configuration
        config_filename="config/server_gyrus_config.yaml"
        logging.debug("configuring and starting gyrii")
        gyrii.config_and_start(config_filename)
        logging.debug("gyrii started")
        while True:
            time.sleep(1)
            ...
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
