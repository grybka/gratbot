
from Gyrus import ThreadedGyrus
import logging
import uuid
from uncertainties import ufloat,umath
from underpinnings.BayesianArray import BayesianArray
from underpinnings.id_to_name import id_to_name
from scipy.optimize import linear_sum_assignment
from underpinnings.MotionCorrection import MotionCorrection
import time
import numpy as np
import cv2 as cv
from scipy.signal import resample
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

#records where objects are relative to me
class OMG_Object:
    def __init__(self,position,size_scale):
        self.position=position #BayesianArray of x,y,z
        self.size_scale=size_scale

class ObjectMapGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None,object_data=[]):
        super().__init__(broker)
        self.display=display
        self.last_track_object_pairing={}
        self.objects={} #id to OMG_Object
        self.motioncorrection=MotionCorrection()
        self.focal_length=1.0/(2*np.tan(np.pi*51.5/360)) #in fraction of image
        self.focal_length_x=self.focal_length

        #TODO persistance
        #how many seconds to keep track of things without any position info
        self.object_persist_time=60.0

        #I have to keep this somewhere
        self.speaker_objectid_pairing={}
        self.objectid_name_pairing={}
        for elem in object_data:
            if "speaker_name" in elem:
                self.speaker_objectid_pairing[elem["speaker_name"]]=elem["id"]
            self.objectid_name_pairing[elem["id"]]=elem["proper_name"]

        #for drawing
        self.next_draw_update=0
        self.draw_update_period=0.5

    def get_keys(self):
        return ["tracks","packets","clock_pulse","track_object_pairing","word_heard"]

    def get_name(self):
        return "ObjectMapGyrus"

    def read_message(self,message):
        self.motioncorrection.read_message(message)
        if 'tracks' in message:
            #TODO image timestamp in motion correction is tricky here
            self.update_from_tracks(message["tracks"])
        if 'track_object_pairing' in message:
            #gives track id -> object id
            self.last_track_object_pairing=message["track_object_pairing"]
        if 'clock_pulse' in message:
            if time.time()>self.next_draw_update:
                self.draw_object_map()
                self.next_draw_update=time.time()+self.draw_update_period
        if 'word_heard' in message:
            #logger.debug(self.speaker_object_pairing)
            self.update_from_word_heard(message["word_heard"])

    def update_from_word_heard(self,wordheard):
        if wordheard["speaker"][0][0] in self.speaker_objectid_pairing:
            id=self.speaker_objectid_pairing[wordheard["speaker"][0][0]]
            heading=wordheard["heading"]
            dist=ufloat(2,1.0)
            theta=ufloat(heading,2*np.pi/20)
            z=dist*umath.cos(theta)
            x=dist*umath.sin(theta)
            y=ufloat(1,1)
            pred=BayesianArray.from_ufloats([x,y,z])
            if id in self.objects:
                self.objects[id].position=self.objects[id].position.updated(pred)
            else:
                self.objects[id]=OMG_Object(pred,0.1)


    def update_from_tracks(self,tracks):
        for track in tracks:
            #only update if its a paired object
            if track["id"] in self.last_track_object_pairing:
                if "spatial_array" not in track:
                    continue
                if track["info"]=="LOST" or track["info"]=="EXITED":
                    continue
                object_id=self.last_track_object_pairing[track["id"]]
                track_pos,size_scale=self.track_to_position(track)
                if object_id in self.objects:
                    #just update
                    self.objects[object_id].position=track_pos
                    self.objects[object_id].size_scale=size_scale
                    #logger.debug("updated position {}".format(track_pos))
                    #logger.debug("size scale {}".format(size_scale))
                else:
                    #its new
                    self.objects[object_id]=OMG_Object(track_pos,size_scale)

    def track_to_position(self,track):
        if "spatial_array" in track:
            #if I've already got depth perception going
            xyz=track["spatial_array"]
            depth=np.linalg.norm(xyz)
            size_scale=depth*(track["bbox_array"][1]-track["bbox_array"][0])/self.focal_length_x
            return BayesianArray(xyz,0.01*np.eye(3)),size_scale

        bbox=track["bbox_array"]
        track_center=track["center"]
        track_center_unc=track["center_uncertainty"]
        #TODO do I include extent here?
        angle_x=ufloat((track_center[0]-0.5)/self.focal_length_x,(track_center_unc[0])/self.focal_length_x)
        angle_x_extent=(bbox[1]-bbox[0])/self.focal_length_x
        angle_y=ufloat((track_center[1]-0.5)/self.focal_length_y,(track_center_unc[1])/self.focal_length_y)
        angle_y_extent=(bbox[3]-bbox[1])/self.focal_length_y
        size_scale=float(max(angle_x_extent,angle_y_extent)*track_center[2])
        dist=ufloat(track_center[2]+size_scale/2,track_center_unc[2])
        #convert to xyz
        x=dist*angle_x
        y=dist*angle_y
        z=dist*umath.cos(angle_x)*umath.cos(angle_y)
        return BayesianArray.from_ufloats([x,y,z]),size_scale


    def draw_object_map(self):
        #emit a led map

        downsampled=np.clip(resample(self.direction_neurons,12),-1,1)
        print(downsampled)

        myrgb=[]
        for i in range(12):
            val=int(100*downsampled[i])
            if val>0:
                my_rgb.append([val,val,val])
            else:
                val=abs(val)
                my_rgb.append([0,0,val])
        broker.publish({"led_command":{"rgb_brightness": my_rgb}},"led_command")
        xsize=640
        ysize=480
        scale=xsize/16 #pixels per meter
        toshow=np.zeros([ysize,xsize,3],np.uint8)
        #inview,almost_inview=self.get_objects_in_view_cone()
        inview=[]
        almost_inview=[]
        for id in self.objects:
            sobj=self.objects[id]
            occupancy=sobj.position.copy()
            #logger.debug("occpancy {}".format(sobj.position))
            #logger.debug("object size scale {}".format(sobj.size_scale))
            #logger.debug("object dist {}".format(np.sqrt(occupancy.vals[0]**2+occupancy.vals[1]**2+occupancy.vals[2])))
            #logger.debug("object dist {}".format(np.linalg.norm(occupancy.vals)))
            extra_cov=(sobj.size_scale/2)**2
            occupancy.covariance[0][0]+=extra_cov
            occupancy.covariance[1][1]+=extra_cov
            occupancy.covariance[2][2]+=extra_cov

            y,x,h2,h1,theta=occupancy.get_error_ellipse(var1=0,var2=2)

            ex=int(y*scale+xsize/2)
            ey=int(x*scale+ysize/2)

            if sobj in inview:
                #logger.debug("inview")
                color = (255, 150, 150)
            elif sobj in almost_inview:
                #logger.debug("almost inview")
                color = (150, 255, 150)
            else:
                color = (150, 150, 150)
            thickness = 2
            theta=-np.degrees(theta)
            cv.ellipse(toshow,(ex,ey),( int(h1*scale),int(h2*scale)),theta,0,360,color,thickness)

            color = (255, 255, 255)
            fontScale = 0.4
            # Line thickness of 2 px
            thickness = 1
            font = cv.FONT_HERSHEY_SIMPLEX
            #text="{} ({})".format(sobj.object_label,id_to_name(sobj.id))
            if id in self.objectid_name_pairing:
                text="{}".format(self.objectid_name_pairing[id])
            else:
                text="*{}".format(id_to_name(id))
            #text="{}".format(sobj.object_label,id_to_name(sobj.id))
            org=( ex,ey)
            toshow = cv.putText(toshow, text, org, font,  fontScale, color, thickness, cv.LINE_AA)
        #cv.arrowedLine(toshow,(320,200),(320,180),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        cv.arrowedLine(toshow,(320,240),(340,240),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        self.display.update_image("Objectmap",toshow)
    #class ObjectMapGyrusObject:
#    def __init__(self,position=np.zeros(3),track_id=None,label=None,size_scale=0):
#        super().__init__(broker)
#        self.id=uuid.uuid1() #or tie to tracker id?
#        #location information
#        self.position=position #BayesianArray of x,y,z
#        self.size_scale=size_scale
#        #how does it look
#        self.appearance_vector=None
#        self.object_label=label #label from tracker
#
#    def update_position(self,position,size_scale):
#        self.position=self.position.updated(position)
#        self.size_scale=size_scale
#
#    def update_linear_motion(self,my_motion):
#        #my_motion should be a BayesianArray
#        self.position-=my_motion
#
#    def update_turn(self,rotmat):
#        xyz=self.position.get_as_ufloat(self)
#        new_xyz=np.matmul(rot_mat,xyz)
#        self.position=BayesianArray.from_ufloats(new_xyz)
#
#
#class ObjectMapGyrus(ThreadedGyrus):
#    def __init__(self,broker,display=None):
#        super().__init__(broker)
#        self.display=display
#
#        #the objects in my map
#        self.objects=[]
#
#        #self.focal_length=640/(2*np.tan(np.pi*73.5/2/180)) #in pixels
#        #self.focal_length=1.0/(2*np.tan(np.pi*73.5/360)) #in fraction of image
#        self.focal_length=1.0/(2*np.tan(np.pi*51.5/360)) #in fraction of image
#        self.focal_length_x=self.focal_length
#        self.focal_length_y=self.focal_length
#        #tracking pose offsets
#        self.last_used_heading=np.array([0,0,0])
#        self.z_gyro_index=0
#        self.y_gyro_index=2
#        self.x_gyro_index=1
#
#        #for identifying objects
#        self.score_cutoff=6.0
#        self.track_object_map={}
#
#        #for drawing
#        self.next_draw_update=0
#        self.draw_update_period=0.5
#
#    def get_keys(self):
#        return ["tracks","packets","clock_pulse"]
#
#    def get_name(self):
#        return "TrackerGyrus"
#
#    def update_with_turn(self,turn,turn_unc):
#        turn=ufloat(-turn_mag,turn_unc)
#        rot_mat=np.array([ [umath.cos(turn),umath.sin(turn),0],[-umath.sin(turn),umath.cos(turn),0],[0,0,1]])
#        for obj in objects:
#            obj.update_turn(rot_mat)
#
#    def get_objects_in_view_cone(self):
#        ret_inview=[]
#        ret_almostinview=[]
#        x_fov=np.pi*51.6/360
#        for obj in self.objects:
#            x_angle=obj.position.vals[0]/obj.position.vals[2]
#            y_angle=obj.position.vals[1]/obj.position.vals[2]
#            #TODO z clipping?
#            if abs(x_angle)<0.5 and abs(y_angle)<0.5:
#                ret_inview.append(obj)
#                ret_almostinview.append(obj)
#            elif abs(x_angle)<1.0 and abs(y_angle)<1.0:
#                ret_almostinview.append(obj)
#        return ret_inview,ret_almostinview
#
#    def update_from_tracks(self,tracks):
#        #first figure out what I -ought- to see
#
#        #for each thing that I ought to see, figure out if I -do- see it
#            #Question do I count tracks that exist but are missing a frame?
#
#        #logger.debug("{} tracks".format(len(tracks)))
#        inview,almost_inview=self.get_objects_in_view_cone()
#        #logger.debug("{} objects".format(len(almost_inview)))
#        #logger.debug("out of {} objects".format(len(self.objects)))
#        cost_matrix=np.zeros([len(tracks),len(almost_inview)])
#        for obj in almost_inview:
#            for track in tracks:
#                score=self.is_this_that(track,obj)
#        row_ind, col_ind = linear_sum_assignment(cost_matrix)
#        leftover_tracks=np.setdiff1d(np.arange(len(tracks)),row_ind)
#        leftover_objs=np.setdiff1d(np.arange(len(almost_inview)),col_ind)
#        #logger.debug("col ind {}".format(col_ind))
#        #logger.debug("other {}".format(np.arange(len(almost_inview))))
#
#        #logger.debug("{} matches, {} unmatched objects, {} unmatched tracks".format(len(row_ind),len(leftover_objs),len(leftover_tracks)))
#        max_assignment_cost=6.0
#        for i in range(len(row_ind)):
#            if cost_matrix[row_ind[i],col_ind[i]]>max_assignment_cost:
#                logger.debug("cost too big, no match {}".format(cost_matrix[row_ind[i],col_ind[i]]))
#                np.append(leftover_tracks,row_ind[i])
#                np.append(leftover_objs,col_ind[i])
#            else:
#                #logger.debug("match")
#                self.track_object_map[tracks[row_ind[i]]["id"]]=obj.id
#                position,size_scale=self.convert_track_pos_to_xyz(tracks[row_ind[i]])
#                obj.update_position(position,size_scale)
#        #deal with unmatched tracks
#        for i in range(len(leftover_tracks)):
#            track=tracks[leftover_tracks[i]]
#            if track["seen_frames"]>5:
#                position,size_scale=self.convert_track_pos_to_xyz(track)
#                logger.debug("New Object at {}".format(position.pretty_str()))
#                self.objects.append(ObjectMapGyrusObject(position=position,size_scale=size_scale,track_id=track["id"],label=track["label"]))
#                logger.debug("I will name it {}".format(id_to_name(self.objects[-1].id)))
#        #deal with unmatched objects
#        for i in range(len(leftover_objs)):
#            if almost_inview[leftover_objs[i]] in inview:
#                obj=almost_inview[leftover_objs[i]]
#                logger.debug("missing object! {}".format(id_to_name(obj.id)))
#            #otherwise it's perepheral, reasonable to miss
#            ...
#
#    def convert_track_pos_to_xyz(self,track):
#        bbox=track["bbox_array"]
#        track_center=track["center"]
#        track_center_unc=track["center_uncertainty"]
#        #TODO do I include extent here?
#        angle_x=ufloat((track_center[0]-0.5)/self.focal_length_x,(track_center_unc[0])/self.focal_length_x)
#        angle_x_extent=(bbox[1]-bbox[0])/self.focal_length_x
#        angle_y=ufloat((track_center[1]-0.5)/self.focal_length_y,(track_center_unc[1])/self.focal_length_y)
#        angle_y_extent=(bbox[3]-bbox[1])/self.focal_length_y
#        size_scale=float(max(angle_x_extent,angle_y_extent)*track_center[2])
#        dist=ufloat(track_center[2]+size_scale/2,track_center_unc[2])
#        #convert to xyz
#        x=dist*angle_x
#        y=dist*angle_y
#        z=dist*umath.cos(angle_x)*umath.cos(angle_y)
#        return BayesianArray.from_ufloats([x,y,z]),size_scale
#
#    def is_this_that(self,track,obj):
#        #returns the likelyhood that an track actually corresponds to an object in my memory
#        chisq=0
#        #if I'm already tracking, cut it some slack
#        if track["id"] in self.track_object_map and self.track_object_map[track['id']]==obj.id:
#            chisq-=1
#        #if the label isn't right, that's a problem
#        if track["label"] != obj.object_label:
#            chisq+1
#        #location part
#        center,size_scale=self.convert_track_pos_to_xyz(track)
#        center.covariance[0][0]+=size_scale*size_scale
#        center.covariance[1][1]+=size_scale*size_scale
#        center.covariance[2][2]+=size_scale*size_scale
#        chisq=obj.position.chi_square_from_pose(center)
#        return chisq
#
#    def read_message(self,message):
#        #I could ask it where to find something?
#        if 'clock_pulse' in message:
#            if time.time()>self.next_draw_update:
#                self.draw_object_map()
#                self.next_draw_update=time.time()+self.draw_update_period
#        if 'tracks' in message:
#            self.update_from_tracks(message["tracks"])
#        if 'packets' in message: #rotation etc
#            #I'm really only interested in the last measurement
#            packet=message['packets'][-1]
#            next_heading=np.array(packet["local_rotation"])
#            accel=np.array(packet["acceleration"])
#            delta_heading=next_heading-self.last_used_heading
#            #I'm assuming it only turns
#            mag_accel=np.linalg.norm(accel)
#            cos_angle=accel[1]/mag_accel
#            sin_angle=accel[2]/mag_accel
#            turn_mag=delta_heading[self.z_gyro_index]*cos_angle-delta_heading[self.y_gyro_index]*sin_angle #in radians
#            #inteperet this as a pose offset
#            self.update_with_turn(turn,turn_unc*turn)
#            self.last_used_heading=next_heading
#
#    def draw_object_map(self):
#        xsize=640
#        ysize=480
#        scale=xsize/8 #pixels per meter
#        toshow=np.zeros([ysize,xsize,3],np.uint8)
#        inview,almost_inview=self.get_objects_in_view_cone()
#        for sobj in self.objects:
#            occupancy=sobj.position.copy()
#            print("object size scale {}".format(sobj.size_scale))
#            print("object dist {}".format(np.sqrt(occupancy.vals[0]**2+occupancy.vals[1]**2+occupancy.vals[2])))
#            extra_cov=(sobj.size_scale/2)**2
#            occupancy.covariance[0][0]+=extra_cov
#            occupancy.covariance[1][1]+=extra_cov
#            occupancy.covariance[2][2]+=extra_cov
#
#            x,y,h1,h2,theta=occupancy.get_error_ellipse(var1=0,var2=2)
#
#            ex=int(y*scale+xsize/2)
#            ey=int(x*scale+ysize/2)
#
#            if sobj in inview:
#                #logger.debug("inview")
#                color = (255, 150, 150)
#            elif sobj in almost_inview:
#                #logger.debug("almost inview")
#                color = (150, 255, 150)
#            else:
#                color = (150, 150, 150)
#            thickness = 2
#            theta=-np.degrees(theta)
#            cv.ellipse(toshow,(ex,ey),( int(h1*scale),int(h2*scale)),theta,0,360,color,thickness)
#
#            color = (255, 255, 255)
#            fontScale = 0.4
#            # Line thickness of 2 px
#            thickness = 1
#            font = cv.FONT_HERSHEY_SIMPLEX
#            text="{} ({})".format(sobj.object_label,id_to_name(sobj.id))
#            #text="{}".format(sobj.object_label,id_to_name(sobj.id))
#            org=( ex,ey)
#            toshow = cv.putText(toshow, text, org, font,  fontScale, color, thickness, cv.LINE_AA)
#        #cv.arrowedLine(toshow,(320,200),(320,180),(255,255,255),1, cv.LINE_AA, 0, 0.3)
#        cv.arrowedLine(toshow,(320,240),(340,240),(255,255,255),1, cv.LINE_AA, 0, 0.3)
#        self.display.update_image("Objectmap",toshow)
