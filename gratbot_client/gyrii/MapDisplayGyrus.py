import cv2 as cv
from Gyrus import ThreadedGyrus
import threading
from underpinnings.BayesianArray import BayesianArray
import time

class MapDisplayGyrus(ThreadedGyrus):

    def __init__(self,broker,shared_objects,display_loop):
        #self.occupancy_map=occupancy_map
        self.display_loop=display_loop
        self.seconds_per_frame=0.1
        self.skip_until_time=0
        self.last_pose=None
        super().__init__(broker,shared_objects)

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
                with self.shared_objects.locks["occupancy_map"]:
                    occupancy_map=self.shared_objects.objects["occupancy_map"]
                    gmap_image=occupancy_map.occupancy_to_image(self.last_pose)
                    start_x=-occupancy_map.resolution*occupancy_map.npoints_x/2
                    end_x=occupancy_map.resolution*occupancy_map.npoints_x/2
                    start_y=-occupancy_map.resolution*occupancy_map.npoints_y/2
                    end_y=occupancy_map.resolution*occupancy_map.npoints_y/2
                with self.shared_objects.locks["graph_map"]:
                    graph_map=self.shared_objects.objects["graph_map"]
                    graph_map.opencv_apply_to_image(gmap_image,start_x,start_y,end_x,end_y)
                self.display_loop.update_image("Map",gmap_image)
                #frontier_image=self.occupancy_map.frontier_to_image()
                #self.display_loop.update_image("Frontier",frontier_image)
                self.skip_until_time=time.time()+self.seconds_per_frame
