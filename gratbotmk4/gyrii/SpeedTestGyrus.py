
import time
import numpy as np
import logging
import uuid
from Gyrus import ThreadedGyrus
from oakd_interface.OakDCalculations import HostSpatialsCalc

class SpeedTestGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.spatial_calc=HostSpatialsCalc(70)
        self.depthFrame=None

    def get_keys(self):
        return ["detections","rotation_vector","depth"]

    def get_name(self):
        return "SpeedTestGyrus"

    def read_message(self,message):
        if "depth" in message:
            self.depth_frame=message["depth"]
        if "detections" in message and len(message["detections"])!=0:
            detections=message["detections"]
            for i in range(len(detections)):
                bbox_array=detections["bbox_array"]
                start=time.time()
                depth=self.spatial_calc.calc_spatials( depthFrame, bbox_array, averaging_method=np.mean)
                logging.debug("time to calculate depth {}".format(start-time.time()))
                logging.debug("spatials are is {}".format(depth))
