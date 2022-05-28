
import time
import numpy as np
import logging
import uuid
from Gyrus import ThreadedGyrus
from oakd_interface.OakDCalculations import HostSpatialsCalc
from underpinnings.LocalPositionLog import LocalPositionLog
from pyquaternion import Quaternion
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class SpeedTestGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.spatial_calc=HostSpatialsCalc(71.86)
        self.position_log=LocalPositionLog()
        self.depthFrame=None

    def get_keys(self):
        return ["detections","rotation_vector","depth"]

    def get_name(self):
        return "SpeedTestGyrus"

    def read_message(self,message):
        self.position_log.read_message(message)
        if "depth_image" in message:
            self.depthFrame=message["depth_image"]
        if "detections" in message and len(message["detections"])!=0:
            detections=message["detections"]
            start=time.time()
            quat=Quaternion(self.position_log.pointing)
            rot=quat.rotation_matrix
            for i in range(len(detections)):
                yxz=detections[i]["spatial_array"]
                xyz=np.array([yxz[1],yxz[0],yxz[2]])
                new_xyz=rot@xyz
            logging.debug("time to calculate rotation {}".format(start-time.time()))


            #if self.depthFrame is None:
            #    return
            #detections=message["detections"]
            #for i in range(len(detections)):
            #    bbox_array=detections[i]["bbox_array"]
            #    start=time.time()
            #    logger.debug("bbox array {}".format(bbox_array))
            #    depth=self.spatial_calc.calc_spatials( self.depthFrame, bbox_array, averaging_method=np.mean)
            #    logging.debug("time to calculate depth {}".format(start-time.time()))
            #    logging.debug("spatials are is {}".format(depth))
            #    logging.debug("onboard spatials are {}".format(detections[i]["spatial_array"]))
