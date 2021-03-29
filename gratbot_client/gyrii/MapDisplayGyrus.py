import cv2 as cv
from Gyrus import ThreadedGyrus
import threading
from underpinnings.BayesianArray import BayesianArray
import time

class MapDisplayGyrus(ThreadedGyrus):

    def __init__(self,broker,occupancy_map,display_loop):
        self.occupancy_map=occupancy_map
        self.display_loop=display_loop
        self.seconds_per_frame=0.1
        self.skip_until_time=0
        self.last_pose=None
        super().__init__(broker)

    def get_keys(self):
        return [ "latest_pose","clock_pulse" ]

    def get_name(self):
        return "MapDisplayGyrus"

    def read_message(self,message):
        if "latest_pose" in message:
            self.last_pose=BayesianArray.from_object(message["latest_pose"])
        if "clock_pulse" in message:
            if self.last_pose is None:
                return
            if message["timestamp"]>self.skip_until_time:
                gmap_image=self.occupancy_map.occupancy_to_image(self.last_pose)
                self.display_loop.update_image("Map",gmap_image)
                self.skip_until_time=time.time()+self.seconds_per_frame
