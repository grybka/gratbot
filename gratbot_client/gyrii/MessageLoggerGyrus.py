from Gyrus import ThreadedGyrus
from Gyrus import Gyrus
import time
import json

import cv2 as cv

class MessageLoggerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        #self.save_filename = "logs/sensor_log_{}.txt".format(self.start_timestr)
        self.save_filename = "logs/sensor_log_latest.txt".format(self.start_timestr)
        self.f=open(self.save_filename,"w")
        super().__init__(broker)

    def on_end(self):
        self.f.close()

    def read_message(self,message):
        #special cases to skip
        #camera images
        if "camera_frame" in message:
            return []
        if "local_map" in message:
            return []
        #otherwise save to log file
        #f=open(self.save_filename,"a")
        try:
            #f.write(json.dumps(message)+"\n")
            self.f.write(json.dumps(message)+"\n")
        except:
            print("Error writing message {}".format(message))
        #f.close()
        return []

    def get_keys(self):
        return [ "latest_pose","pose_measurement","pose_offset","drive/motors_active","notification","velocity_measurement","pose_certainty_lost","motor_command","video_offset","visual_tracker_objects" ]

    def get_name(self):
        return "MessageLogger"

class ImageLoaderGyrus(Gyrus):
    def __init__(self):
        self.prefix="../"
        pass

    def read_message(self,message):
        ret=[]
        if "saved_image" in message:
            frame=cv.imread(self.prefix+message["saved_image"])
            ret.append({"timestamp":message["timestamp"], "camera_frame": frame})
        return ret
