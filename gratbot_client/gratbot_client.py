#gratbot client

import sys
import numpy as np
import threading
import cv2 as cv
import logging
import time
import os
import traceback
import curses

simulation_mode=False
#simulation_mode=True

#sys.path.append('../gratbot_client')
sys.path.append('gyrii')
sys.path.append('gyrii/behaviors')
#from GratbotCommsMk2 import GratbotCommsMk2
#from GratbotUV4LConnection import GratbotUV4LConnection
from GratbotUV4LToBrokerThread import GratbotUV4LToBrokerThread
from Gyrus import VideoDisplay,GyrusSharedObjects
from Gyrus import ThreadedGyrus
from MessageBroker import MessageBroker
from CompassGyrus import Compass
from PoseTrackerGyrus import PoseTrackerGyrus
from BehaviorGyrus import BehaviorGyrus
from CommsGyrus import CommsGyrus
from MotionGyrus import MotionEstimationGyrus
from initial_behavior import get_behavior
from MessageLoggerGyrus import MessageLoggerGyrus
#from LocalMapGyrus import LocalMapGyrus
from LocalMapGyrus2 import LocalMapGyrus
from FloorWallGyrus import FloorDetectorGyrus
from MapDisplayGyrus import MapDisplayGyrus
from ObjectTagger2 import ObjectTaggerGyrus
#from VisualTracker2 import VisualTrackerGyrus
from VisualTracker3 import VisualTrackerGyrus
from ObjectMap import ObjectMapGyrus
#from VisualOdometer import VisualOdometerGyrus
from VisualOdometerConv import VisualOdometerConvGyrus
from VisualMotionTrackingGyrus import VisualMotionTrackingGyrus
from LidarMotion import LidarICPTracker
from ScaledDownVisualGyrus import ScaledDownVisualGyrus
from LowLevelSensorPredictor import LowLevelSensorPredictor
from VisualMotionCalibGyrus import VisualMotionCalibGyrus

from gyrii.underpinnings.OccupancyMap2 import OccupancyMap2,LocalizationException
from gyrii.underpinnings import GratbotLogger
from gyrii.underpinnings.GratbotLogger import gprint,global_screen
from gyrii.underpinnings.GraphMap import GraphMap


#from FaceRecognizer import FaceRecognizer
#from ObjectTagger import ObjectTaggerGyrus
import yaml
import queue

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)

class DisplayLoop(VideoDisplay):
    def __init__(self):
        self.window_images={}
        self.open_windows=[]
        self.keep_going=True
        self.frame_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

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

    def _run(self):
        while self.keep_going:
            self.one_loop()
        logging.warning("closing window ")
        cv.destroyAllWindows()

    def stop(self):
        self.keep_going=False
        self.thread.join()
display_loop=DisplayLoop()

class ClockLoop:
    def __init__(self,broker):
        self.broker=broker
        #self.clock_pulse_period=0.05
        self.clock_pulse_period=0.1
        self.keep_going=True
        self.frame_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while self.keep_going:
            time.sleep(self.clock_pulse_period)
            broker.publish({"timestamp": time.time(),"clock_pulse": self.clock_pulse_period},"clock_pulse")

    def stop(self):
        self.keep_going=False
        self.thread.join()

# connect to bot controls
#logging.info("Connecting to Gratbot comms")
#gratbot_comms = GratbotCommsMk2("10.0.0.4", 9999)
#gratbot_comms.set(["ultrasonic_sensor","update_frequency"],4)

#connect to video
#video_connection=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")

#This is for text input
textinput=queue.Queue()

#This stores the message passing
broker=MessageBroker()

#This stores objects between gyrii.  A good opportunity for threadlock
shared_objects=GyrusSharedObjects()
map_length=20.0
#npoints_x=400
npoints_x=200
#npoints_y=400
npoints_y=200
#map_resolution=0.05
map_resolution=0.1
lattice_length=0.25 #how close mapgraph nodes are to each other
occupancy_map=OccupancyMap2(map_resolution,npoints_x,npoints_y)
shared_objects.add_object("occupancy_map",occupancy_map)
graphmap=GraphMap(map_length,lattice_length)
shared_objects.add_object("graph_map",graphmap)



#TODO set up behaviors
#TODO set up gyrii
gyrii=[]
gyrii.append(CommsGyrus(broker,simulation_mode=simulation_mode))
#gyrii.append(Compass(broker))
gyrii.append(MessageLoggerGyrus(broker))
#gyrii.append(PoseTrackerGyrus(broker))
#gyrii.append(MotionEstimationGyrus(broker))

#localmapgyrus=LocalMapGyrus(broker,shared_objects,display_loop=display_loop)
#localmapgyrus.state="both"
#gyrii.append(localmapgyrus)

gyrii.append(BehaviorGyrus(broker,get_behavior(),pass_kwargs={"broker": broker,"shared_objects": shared_objects,"text_input": textinput}))
gyrii.append(VisualTrackerGyrus(broker,display_loop)) #This One
gyrii.append(VisualMotionCalibGyrus(broker,"config/visualmotioncalibgyrus.state"))

#gyrii.append(FloorDetectorGyrus(broker,checkpoint_fname="floor_detector_sliced.pt",debugshow=False,min_distance=0.1))
#gyrii.append(MapDisplayGyrus(broker,localmapgyrus.gridmap,display_loop))

#gyrii.append(MapDisplayGyrus(broker,shared_objects,display_loop)) #this one
#gyrii.append(LowLevelSensorPredictor(broker))  #this one


#gyrii.append(ObjectMapGyrus(broker,display_loop=display_loop))
#gyrii.append(VisualOdometerGyrus(broker,display=display_loop))
#gyrii.append(FaceRecognizer(display_loop))
#gyrii.append(VisualOdometerConvGyrus(broker,display=display_loop))
#gyrii.append(VisualMotionTrackingGyrus(broker,display=display_loop))
#gyrii.append(LidarICPTracker(broker,display=display_loop))
#gyrii.append(ScaledDownVisualGyrus(broker,display=display_loop))
#gyrii.append(ObjectTaggerGyrus(broker,display=display_loop))

#last_update=time.time()
#update_spacing=0.03 #in seconds

#start the vidio
video_thread=GratbotUV4LToBrokerThread(broker,"http://10.0.0.4:8080/stream/video.mjpeg",display_loop,simulation_mode=simulation_mode)

#go_time=time.time()

from curses_test import SplitScreenDisplay
from curses import wrapper

def main_loop(stdscr):


    try:
        display=SplitScreenDisplay(stdscr)
        global_screen.append(display)
        gprint("display is up")

        #Load existing config file into the gyrii
        config_filename="config/gyrus_config.yaml"
        try:
            with open(config_filename,'r') as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                for g in gyrii:
                    g.load_config(data)
                f.close()
        except:
            print("Unable to load gyrus config file")


        for g in gyrii:
            g.start_thread()

        print("beginning loop")
        clock_loop=ClockLoop(broker)



        #This is the behavior execution loop
        while True:
            res=display.allow_input()
            if res is not None:
                display.add_text_to_bottom(res)
                if res=="quit":
                    break
                textinput.put(res)

            #display_loop.one_loop()
            #time.sleep(0.1)
            #this could be its own thread and I could do a full CLI here
            #if time.time()>go_time:
            #    broker.publish({"timestamp": time.time(),"clock_pulse": 0.1},"clock_pulse")
            #    go_time=time.time()+0.1

    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, saving config and exiting")
    except Exception as e:
        print("Exception: {}".format(e))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        traceback.print_exc(file=sys.stdout)
    finally:
        display.end()
        clock_loop.stop()
        video_thread.stop()
        cv.destroyAllWindows()
        display_loop.stop()
        for g in gyrii:
            g.should_quit=True
        #save gyri info
        config_object={}
        for g in gyrii:
            config_object.update(g.save_config())
            g.join()
        #start_timestr = time.strftime("%Y%m%d-%H%M%S")
        #config_filename="config/gyrus_config_{}.yaml".format(start_timestr)
        config_filename="config/gyrus_config_latest.yaml"
        with open(config_filename,'w') as f:
            data = yaml.dump(config_object,f)
            f.close()

wrapper(main_loop)
