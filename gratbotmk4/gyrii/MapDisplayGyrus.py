from Gyrus import ThreadedGyrus
from underpinnings.id_to_name import id_to_name
import logging,time
import numpy as np
import cv2
from underpinnings.LocalPositionLog import LocalPositionLog
from pyquaternion import Quaternion

logger=logging.getLogger(__name__)
#logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

class MapDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None,mode="show_tracks"):
        super().__init__(broker)
        self.display=display
        self.mode=mode
        self.position_log=LocalPositionLog()

    def get_keys(self):
        return ["tracks","rotation_vector"]

    def get_name(self):
        return "MapDisplayGyrus"

    def read_message(self,message):
        self.position_log.read_message(message) #handles rotation vector and motors
        if "tracks" in message and self.mode=="show_tracks":
            self.update_display(message["tracks"])

    def update_display(self,tracks):
        map_size_x=640
        map_size_y=640
        dist_scale=80*1e-3 #pixels per mm
        image=np.zeros((map_size_y,map_size_x,3),np.uint8)

        quat=Quaternion(self.position_log.pointing)
        rot=quat.rotation_matrix
        #logger.info("ponting {}".format(self.pointing))
        forward_vec=np.array([0,0,20])
        left_vec=np.array([0,20,0])
        mapvec=rot@forward_vec

        startpt=[320,320]
        endpt=[int(320+mapvec[1]),int(320+mapvec[0])]
        cv2.arrowedLine(image,startpt,endpt,(255,255,255),1, cv2.LINE_AA, 0, 0.3)
        for track in tracks:
            xyz=track["map_position"]
            #logger.debug("xyz {}".format(xyz))
            centr=[int(xyz[1]*dist_scale+map_size_y/2),int(xyz[0]*dist_scale+map_size_x/2)]
            cv2.circle(image,centr,6, (255,0,0), 1)
            cv2.putText(image,track["label"],[centr[0],centr[1]-10],cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)

        self.display.update_image("map_display",image)
