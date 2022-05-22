#make semi-persistent objects from tracks

import logging
import uuid
import time
import numpy as np
from uncertainties import ufloat
from uncertainties.umath import *
from pyquaternion import Quaternion
from underpinnings.MotionCorrection import MotionCorrectionRecord
from Gyrus import ThreadedGyrus
from underpinnings.BayesianArray import BayesianArray
from underpinnings.id_to_name import id_to_name
from scipy.optimize import linear_sum_assignment
from underpinnings.LocalPositionLog import LocalPositionLog
import cv2

#takes a collection of tracks and infers the existance of objects that persist even when they leave the visual field

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
#logger.setLevel(logging.INFO)

class STMObject:
    def __init__(self,position,track,timestamp):
        self.id=uuid.uuid4()
        self.position=position #BayesianArray
        self.height=BayesianArray(ndim=1)
        self.height.covariance*=100
        self.label=track["label"]
        #TODO I need some way to make an object preliminary until verified somehow
        #Options:
        self.seen_for_a_while=False
        self.first_seen=timestamp
        self.last_seen=timestamp
        #  preliminary until a certain time
        #  preliminary until seen from several angles
        ...

    def update_with_trackpos(self,trackpos,timestamp,label):
        a_while=3 #seconds
        if timestamp-self.first_seen>a_while:
            self.seen_for_a_while=True
        self.position=self.position.updated(trackpos)
        #logger.debug("new pos {}".format(self.position))
        #self.position.min_covariance([0.01,0.01,0.01]) #10 cm covariance
        self.position.min_covariance([1,1,1]) #10 cm covariance
        self.last_seen=timestamp
        self.label=label

    def is_preliminary(self):
        return not self.seen_for_a_while

class ShortTermObjectMemory(ThreadedGyrus):
    def __init__(self,broker,display):
        super().__init__(broker)
        self.position_log=LocalPositionLog()
        self.display=display
        self.track_id_object_map={}
        #self.object_track_id_map={}
        self.objects={}
        self.motion_corrector=MotionCorrectionRecord()
        #self.pointing=[1,0,0,0] #the Quaternion that represents my orientation compared to the map
        self.track_info={} #record of tracks that were seen
        self.last_update=time.time()
        self.recalc_period=0.4
        self.n_track_messages_integrated=0
        self.last_depth=None
        self.ok_labels=["sports ball"]
        self.focus_object=None

    def get_keys(self):
        return ["rotation_vector","tracks","servo_response","gyrus_config","clock_pulse","motor_report","depth"]

    def get_name(self):
        return "ShortTermObjectMemory"

    def update_focus(self):
        if self.focus_object is not None:
            ...
            #if I'm already focused on something, why change?
            #TODO if I get bored with it
        else: #No focus, let's find a new one
            for objid in self.objects:
                #TODO have some reason
                self.focus_object=objid
        #logger.debug("focus object is {}".format(self.focus_object))
        if self.focus_object is not None:
            obj=self.objects[self.focus_object]
            message={"object_relative_position": obj.position.vals} #on the map
            quat=Quaternion(self.position_log.pointing)
            invrot=quat.conjugate.rotation_matrix
            message["object_pointing"]=invrot@obj.position.vals #relative to facing
            message["object_height"]=obj.height.vals[0]
            for trackid in self.track_id_object_map:
                if self.track_id_object_map[trackid]==self.focus_object:
                    message["track_id"]=trackid
            self.broker.publish(message,["st_object_report"])
            #logger.debug("message {}".format(message))

    def is_object_in_frustrum(self,obj):
        quat=Quaternion(self.position_log.pointing)
        invrot=quat.conjugate.rotation_matrix
        mypos=invrot@obj.position.vals
        #forward is 0,0,1
        #viewing frustrum
        theta=0.5*(55/360)*(2*np.pi)
        bottom_plane=[sin(theta),0,cos(theta)]
        top_plane=[-sin(theta),0,cos(theta)]
        theta=0.5*(69/360)*(2*np.pi)
        left_plane=[0,sin(theta),cos(theta)]
        right_plane=[0,-sin(theta),cos(theta)]
        if np.dot(mypos,bottom_plane)<0:
            logger.info("{} excluded by bottom plane".format(id_to_name(obj.id)))
            return False
        if np.dot(mypos,top_plane)<0:
            logger.info("{} excluded by top plane".format(id_to_name(obj.id)))
            return False
        if np.dot(mypos,right_plane)<0:
            logger.info("{} excluded by right plane".format(id_to_name(obj.id)))
            return False
        if np.dot(mypos,left_plane)<0:
            logger.info("{} excluded by left plane".format(id_to_name(obj.id)))
            return False
        return True

    def guess_depth(self,track):
        if self.last_depth is None:
            return None
        # With 400P mono camera resolution where HFOV=71.9 degrees
        #focal_length_in_pixels = 640 * 0.5 / tan(71.9 * 0.5 * PI / 180) = 441.25
        focal_length_in_pixels = 441.25
        baseline=7.5 #cm
        bbox=track["last_detection_bbox"]
        depthimage=self.last_depth["depth_image"]
        #logger.debug("depth bbox {}".format(bbox))
        #logger.debug("depth image shape {}".format(depthimage.shape))
        xstart=int(bbox[0]*depthimage.shape[1])
        xstop=int(bbox[1]*depthimage.shape[1])
        ystart=int(bbox[2]*depthimage.shape[0])
        ystop=int(bbox[3]*depthimage.shape[0])
        in_bbox=depthimage[ystart:ystop,xstart:xstop,0]
        nz=np.nonzero(in_bbox) #zero means no reading
        if len(nz)==0:
            return None
        #av_disp=np.mean(in_bbox[nz])
        av_disp=np.median(in_bbox[nz])
        st_disp=np.std(in_bbox[nz])
        depth=focal_length_in_pixels*baseline/av_disp
        deptherr=depth*st_disp/av_disp
        #logger.debug("depth is {} pm {}".format(depth,deptherr))
        if depth<10 or depth>400:
            logger.warning("unusual depth given: {}".format(depth))
            return ufloat(depth,400)/100
        return ufloat(depth,deptherr)/100

    def get_track_position_knowing_size(self,track,timestamp,object):
        angle_unc=4*np.pi/360 #flat 4 degree uncertainty.  Could think harder about this
        udist=self.guess_depth(track) #estimate from depth perception
        bbox=track["last_detection_bbox"]
        ahp=bbox[3]-bbox[2]
        height_ratio=tan((ahp)*(55/360)*(2*np.pi))
        dist_from_height=object.height.get_as_ufloat()[0]/height_ratio #estimate from height
        #now backwards
        height_guess=udist*height_ratio
        #update my thinging about height
        #this doesn't belong in here...
        object.height=object.height.updated(BayesianArray.from_ufloats([height_guess]))
        object.height.min_covariance([ (object.height.vals[0]*0.1)**2 ]) #5 cm resolution
        #resolvemy distance belief
        u1=udist.n
        u2=dist_from_height.n
        w1=1/(udist.s**2)
        w2=1/(dist_from_height.s**2)
        new_u=(u1*w1+u2*w2)/(w1+w2)
        new_s=1/sqrt(w1+w2)
        udist=ufloat(new_u,new_s)



        #camera FOV hardcoded.  Awkward
        pitch=(float(track["center"][1])-0.5)*(55/360)*(2*np.pi)
        upitch=ufloat(pitch,angle_unc)
        yaw=-(float(track["center"][0])-0.5)*(69/360)*(2*np.pi)
        uyaw=ufloat(yaw,angle_unc)

        my_x=udist*cos(upitch)*sin(uyaw)
        my_y=udist*cos(uyaw)*sin(upitch)
        my_z=udist*cos(upitch)*cos(uyaw)

        my_xyz=np.array([my_y,my_x,my_z])

        #rotate into map frame
        quat=Quaternion(self.position_log.pointing)
        rot=quat.rotation_matrix
        new_xyz=rot@my_xyz
        #z is now up
        return BayesianArray.from_ufloats(new_xyz)


    def get_track_position(self,track,timestamp):
        angle_unc=4*np.pi/360 #flat 4 degree uncertainty.  Could think harder about this

        udist=self.guess_depth(track)
        if udist is None:
            udist=ufloat(2.0,2.0)

        #camera FOV hardcoded.  Awkward
        pitch=(float(track["center"][1])-0.5)*(55/360)*(2*np.pi)
        upitch=ufloat(pitch,angle_unc)
        yaw=-(float(track["center"][0])-0.5)*(69/360)*(2*np.pi)
        uyaw=ufloat(yaw,angle_unc)

        my_x=udist*cos(upitch)*sin(uyaw)
        my_y=udist*cos(uyaw)*sin(upitch)
        my_z=udist*cos(upitch)*cos(uyaw)

        my_xyz=np.array([my_y,my_x,my_z])

        #rotate into map frame
        quat=Quaternion(self.position_log.pointing)
        rot=quat.rotation_matrix
        new_xyz=rot@my_xyz
        #z is now up
        return BayesianArray.from_ufloats(new_xyz)

    def match_to_object(self,track,timestamp,objects,not_objects):
        chisq_cutoff=100
        trackpos=self.get_track_position(track,timestamp)
        best_obj=None
        best_chisq=chisq_cutoff
        for objid in objects:
            if objid in not_objects:
                continue
            chisq=0
            obj=self.objects[objid]
            if obj.label!=track["label"]:
                chisq+=50
            distance_chisq=trackpos.chi_square_from_pose(obj.position)
            chisq+=distance_chisq
            if chisq<best_chisq:
                best_obj=obj
                best_chisq=chisq
        return best_obj

    def update_object_from_track(self,id,track,timestamp):
        #this is assuming it didn't move  maybe should fix this
        trackpos=self.get_track_position_knowing_size(track,timestamp,self.objects[id])
        if id not in self.objects:
            logger.warning("asking for object id that doesn't exist! {}".format(id))
            return
        self.objects[id].update_with_trackpos(trackpos,timestamp,track["label"])

    def update_track_info(self,tracks,timestamp):
        self.n_track_messages_integrated=self.n_track_messages_integrated+1
        for track in tracks:
            if track["label"] not in self.ok_labels:
                continue
            if track["info"]=="LOST":
                self.track_info[track["id"]]=track
                self.track_info[track["id"]]["timestamp"]=timestamp
            elif track["info"]=="DETECTED":
                ####

                ####
                if track["id"] in self.track_info:
                    if self.track_info[track["id"]]["info"] != "LOST":
                        self.track_info[track["id"]]=track
                        self.track_info[track["id"]]["timestamp"]=timestamp
                else:
                    self.track_info[track["id"]]=track
                    self.track_info[track["id"]]["timestamp"]=timestamp
            else:
                if track["id"] in self.track_info:
                    if self.track_info[track["id"]]["info"] == "LOST" and self.track_info[track["id"]]["info"] == "DETECTED":
                        continue
                    self.track_info[track["id"]]=track
                    self.track_info[track["id"]]["timestamp"]=timestamp

    def handle_tracks(self):
        #first remove tracks that are lost
        unmatched_tracks=list(self.track_info.keys())
        assigned_objects=[] #how many assigned this round
        for trackid in self.track_info:
            track=self.track_info[trackid]
            if track["info"]=="LOST":
                if track["id"] in self.track_id_object_map:
                    del self.track_id_object_map[trackid]
                unmatched_tracks.remove(trackid)
        #So if I'm not moving too much, I should ask if anything is missing
        objects_to_see=[]
        for objid in self.objects:
            if self.is_object_in_frustrum(self.objects[objid]):
                objects_to_see.append(objid)
        tracks_to_match=[ self.track_info[x]["id"] for x in self.track_info]
        #first exclude tracks I have already mateched
        for trackid in self.track_id_object_map:
            if trackid in unmatched_tracks:
                track=self.track_info[trackid]
                timestamp=track["timestamp"]
                objid=self.track_id_object_map[trackid]
                self.update_object_from_track(objid,track,timestamp)
                assigned_objects.append(objid)
                if objid in objects_to_see:
                    objects_to_see.remove(objid)
                unmatched_tracks.remove(trackid)
        #now deal with tracks that don't have a match
        for trackid in unmatched_tracks:
            track=self.track_info[trackid]
            timestamp=track["timestamp"]
            #but how do I make sure I don't reuse an object??
            match=self.match_to_object(track,timestamp,objects_to_see,assigned_objects)
            if match is not None:
                objid=match.id
                self.track_id_object_map[trackid]=objid
                objects_to_see.remove(objid)
            else:
                #this doesn't match anything I know about.  I should try to match
                #with all the objects in memory (except whot was already tried)
                match=self.match_to_object(track,timestamp,self.objects.keys(),objects_to_see+assigned_objects)
                if match is not None:
                    objid=match.id
                    self.track_id_object_map[trackid]=objid
                else:
                    #create new object I guess
                    trackpos=self.get_track_position(track,timestamp)
                    new_obj=STMObject(trackpos,track,timestamp)
                    self.objects[new_obj.id]=new_obj
                    self.track_id_object_map[trackid]=new_obj.id
                    logger.debug("new object {}".format(id_to_name(new_obj.id)))
                    assigned_objects.append(new_obj.id)
        #now deal with objects that don't have a match
        for objid in objects_to_see:
            logger.debug("{} the {} is missing".format(id_to_name(objid),self.objects[objid].label))
        #for trackid in self.track_id_object_map:
        #    logger.debug("Track {} matched to {}".format(id_to_name(trackid),id_to_name(objid)))

    def read_message(self,message):
        self.position_log.read_message(message)
        #if "packets" in message:
        #    packet=message["packets"][-1]
        #    self.pointing=packet["rotation_vector"]
        #if "left_motor" in message:
        #    self.handle_motor_update(message)
        if "tracks" in message:
            self.update_track_info(message["tracks"],message["timestamp"])
        if "depth_image" in message:
            self.last_depth=message
        if "clock_pulse" in message:
            if time.time()-self.last_update>self.recalc_period and self.n_track_messages_integrated>0:
                self.last_update=time.time()
                self.update_expectations_from_motion()
                #logger.debug("{} track messages integrated".format(self.n_track_messages_integrated))
                self.handle_tracks()
                #Reset track info
                self.track_info={}
                self.n_track_messages_integrated=0
                #clear spurious objects
                self.clear_spurious_objects()
                #update focus
                self.update_focus()
                #Draw to screen
                self.draw_image()

    def update_expectations_from_motion(self):
        local_pos=self.position_log.offset
        for objid in self.objects:
            self.objects[objid].position-=local_pos
        self.position_log.reset_offset()

    def clear_spurious_objects(self):
        prelim_persist_time=3 #seconds
        for objid in list(self.objects.keys()):
            obj=self.objects[objid]
            if obj.is_preliminary():
                if time.time()-obj.last_seen>prelim_persist_time: #I haven't seen it, must have imagined it
                    logger.debug("Object {} the {} deemed to be my imagination".format(objid,obj.label))
                    del self.objects[objid]
                    #if I was focused on it, remove
                    if self.focus_object==objid:
                        self.focus_object=None

    def draw_image(self):
        image=np.zeros((640,640,3),np.uint8)
        quat=Quaternion(self.position_log.pointing)
        rot=quat.rotation_matrix
        #logger.info("ponting {}".format(self.pointing))
        forward_vec=np.array([0,0,20])
        left_vec=np.array([0,20,0])
        mapvec=rot@forward_vec
        #mapleftvec=rot@left_vec
        dist_scale=80 #pixels per meter
        startpt=[320,320]
        endpt=[int(320+mapvec[1]),int(320+mapvec[0])]
        cv2.arrowedLine(image,startpt,endpt,(255,255,255),1, cv2.LINE_AA, 0, 0.3)

        for objid in self.objects:
            obj=self.objects[objid]
            xyz=obj.position.vals
            centr=[int(xyz[1]*dist_scale+320),int(xyz[0]*dist_scale+320)]
            cv2.circle(image,centr,6, (255,0,0), 1)
            cv2.putText(image,id_to_name(objid),centr,cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(image,obj.label,[centr[0],centr[1]-10],cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
        self.display.update_image("quattest",image)
