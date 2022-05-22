
import logging
import uuid
import time
from Gyrus import ThreadedGyrus

def get_track_with_id(id,tracks):
    for track in tracks:
        if track["id"]==id:
            return track
    return None

class PointingErrorGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tracked_track_id=None
        self.track_height=1
        self.mode="POINTING_ONLY"
        self.target_distance=1.0 #how far I'm trying to keep my object

    def get_keys(self):
        return ["st_object_report","tracks","clock_pulse","PointingErrorGyrusConfig"]

    def get_name(self):
        return "PointingErrorGyrus"

    def read_message(self,message):
        if "PointingErrorGyrusConfig" in message["keys"]:
            if "mode" in message:
                if message["mode"] in ["POINTING_AND_DISTANCE","POINTING_ONLY","OFF"]:
                    self.mode==message["mode"]
                else:
                    logger.warning("Mode {} not allowed for PointingErrorGyrus".format(message['mode']))
        if "st_object_report" in message["keys"]:
            if "track_id" in message:
                self.tracked_track_id=message["track_id"]
                self.track_height=message["object_height"]
            else:
                #object is off screen or track lost
                p=message["object_pointing"]
                plen=np.linalg.norm(p)
                yaw_error=-np.asin(p[1]/plen)
                pitch_error=-np.asin(p[0]/plen)
                #TODO, if I can't see it, should I really be moving forward?
                #probably not, so report no distance error
                self.report_error(float(yaw_error),float(pitch_error),0)
        if "tracks" in message["keys"]:
            if self.tracked_track_id is not None:
                xerror_signal,yerror_signal,disterr=self.get_track_error(message)
                if xerror_signal is None:
                    return #track has been lost
                if self.mode=="POINTING_AND_DISTANCE":
                    self.report_error(float(xerror_signal),float(yerror_signal),float(disterr))
                elif self.mode=="POINTING_ONLY":
                    self.report_error(float(xerror_signal),float(yerror_signal),0)
                else:
                    ... #no need to report if off


    def report_error(self,xerror,yerror,disterror=0):
        #xerror and yerror should be roughly in degrees
        #disterror should be roughly in meters
        #logger.debug("Reporting error {} {}".format(xerror,yerror))
        message_out={"timestamp": time.time(),"pointing_error_x": float(xerror),"pointing_error_y": float(yerror), "distance_error": float(disterror)}
        self.broker.publish(message_out,["pointing_error_x","pointing_error_y","distance_error"])

    def get_track_error(self,message):
        track=get_track_with_id(self.tracked_track_id,message["tracks"])
        if track is None:
            return None,None
        if track["info"]=="LOST":
            logger.info("Track lost")
            self.tracked_object_id=None
            return None,None
        else:
            logger.debug("track status {}".format(track["info"]))
        yposition_at_image=track["center"][1]
        xposition_at_image=track["center"][0]
        bbox=track["last_detection_bbox"]
        ahp=bbox[3]-bbox[2]
        height_ratio=tan((ahp)*(55/360)*(2*np.pi))
        dist=self.track_height/height_ratio #estimate from height

        yerror_signal=(float(track["center"][1])-0.5)*(55/360)*(2*np.pi) #in radians
        xerror_signal=-(float(track["center"][0])-0.5)*(69/360)*(2*np.pi) #in radians
        disterror=dist-self.target_distance

        return -xerror_signal,-yerror_signal,disterror
