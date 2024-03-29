#no opencv tracker

# will track only one of each type
import cv2
import uuid
import logging
from collections import deque
from underpinnings.id_to_name import id_to_name
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
from filterpy.common import kinematic_kf,Q_discrete_white_noise

from Gyrus import ThreadedGyrus

logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.WARNING)
logger.setLevel(logging.INFO)

#TODO if something moved plausibly offscreen, then it should persist longer


def get_closest_value(timestamp,mylist):
        #for mylist ordered by [time,value], return the value that is closest to the input timestamp
        if len(mylist)==0:
            return None
        if timestamp<=mylist[0][0]:
            return mylist[0][1]

        first_val=mylist[0]
        second_val=None
        for val in mylist:
            if timestamp>val[0]:
                second_val=val
            else:
                first_val=val
        if abs(first_val[0]-timestamp)<abs(second_val[0]-timestamp):
            return first_val[1]
        else:
            return second_val[1]


class MotionCorrection: #to correct image frames from heading changes
    def __init__(self,max_recent_history=20):
        self.max_recent_history=max_recent_history
        self.gyros=deque([],maxlen=self.max_recent_history)
        self.accel=np.array([0,0,10]) #for gravity
        self.headings=deque([],maxlen=self.max_recent_history) #from gyro integration
        #self.last_used_heading=0
        self.last_used_heading=np.array([0,0,0])
        self.angle_heading_slope=-1.515
        self.angle_ygyro_slope=-1.6
        self.z_gyro_index=0
        self.y_gyro_index=2
        self.x_gyro_index=1

    def read_message(self,message):
        if 'packets' in message: #rotation, etc
            if len(self.headings)==0: #for the first packet received
                #self.headings.append([message['packets'][0]["gyroscope_timestamp"],0 ])
                self.headings.append([message['packets'][0]["gyroscope_timestamp"],np.array([0,0,0]) ])
            for packet in message["packets"]:
                #self.gyros.append( [packet["gyroscope_timestamp"],packet["gyroscope"]])
                self.accel=0.8*self.accel+0.2*np.array(packet["acceleration"])
                    #TODO I could do a fancier integration
                #next_heading=self.headings[-1][1]+np.array(packet["gyroscope"])*(packet["gyroscope_timestamp"]-self.headings[-1][0])
                next_heading=np.array(packet["local_rotation"])
                self.headings.append( [packet["gyroscope_timestamp"],next_heading])

    def get_offset_and_update(self,image_timestamp):
        if len(self.headings)==0:
            return 0,0
        closest_heading_vec=get_closest_value(image_timestamp,self.headings)
        delta_heading=closest_heading_vec-self.last_used_heading
        self.last_used_heading=closest_heading_vec #this is the update part
        #offset=delta_heading*self.angle_heading_slope
        #return offset
        #offset_x=delta_heading[self.z_gyro_index]*self.angle_heading_slope
        #TODO figure out how to line this up with gravity
        mag_accel=np.linalg.norm(self.accel)
        cos_angle=self.accel[1]/mag_accel
        sin_angle=self.accel[2]/mag_accel
        turn_mag=delta_heading[self.z_gyro_index]*cos_angle-delta_heading[self.y_gyro_index]*sin_angle
        #offset_x=delta_heading[self.z_gyro_index]*self.angle_heading_slope
        offset_x=turn_mag*self.angle_heading_slope
        offset_y=delta_heading[self.x_gyro_index]*self.angle_ygyro_slope
        return offset_x,offset_y

focal_length = 1/(2*np.tan(73.5/2/180*np.pi))  #multiply by image size


class TrackerGyrusTrackedObject:
    def __init__(self):
        self.id=uuid.uuid1()

        self.info="NEW"
        self.defunct=False
        self.subimage=[]

        #info from detections
        self.last_label=None
        self.last_depth=None
        self.last_det_bbox=None

        #infermation for height and distance inference
        self.base_height=1
        self.base_width=1

        #tracking misses
        self.last_success=True
        self.frames_without_detection=0
        self.frames_with_detection=1
        #kalman filtering
        #x and y in full span of the camera
        #z is in meters and only updated with depth
        dt=1/30.0
        self.kfx=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfy=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfz=kinematic_kf(dim=1, order=1, dt=dt, order_by_dim=False)
        self.kfz.x=np.array([[1.0],[0.0]]) #because z=0 can cause problems
        self.kfx.P=1e9*np.eye(2) #covariance
        self.kfy.P=1e9*np.eye(2) #covariance
        self.kfz.P=1e9*np.eye(2) #covariance
        #From experience, if I had a successful detection,
        #the object isn't moving super fast
        #plus if the velocity covariance is the same as the position,
        #then my first detection will give it an unrealistic expectation
        #of knowing the velocity
        self.kfx.P[1][1]=0.1**2
        self.kfy.P[1][1]=0.05**2
        self.kfz.P[1][1]=0.1**2

    def update_kf_from_time(self,dt):
        #assume 2.3 pixel creep per second
        my_variance=4
        my_variance=0.5
        self.kfx.Q=Q_discrete_white_noise(2,dt=dt,var=my_variance)
        self.kfy.Q=Q_discrete_white_noise(2,dt=dt,var=my_variance)
        #assume 1 cm creep per second
        self.kfz.Q=Q_discrete_white_noise(2,dt=dt,var=0.01**2)
        self.kfx.predict()
        self.kfy.predict()
        self.kfz.predict()

    def update_with_detection(self,det,image,include_subimage=False):
        box=self.get_det_bbox(det)
        if "spatial_array" in det:
            self.last_depth=det["spatial_array"][2]/1000 #in m
        else:
            self.last_depth=1
        self.last_det_bbox=box
        self.last_label=det['label']
        x_update,y_update=box[0]/image.shape[1],box[1]/image.shape[0]
        #print("x update, y update {},{} for {}".format(x_update,y_update,det["label"]))
        #assume 2 pixel resolution for a detection
        velocity_onesigma=1 #m/s
        view_angle=1.6 #radians
        #xy_onesigma=(velocity_onesigma/self.kfz.x[0][0])/view_angle
        self.kfx.H=np.array([[1.,0.]])
        self.kfy.H=np.array([[1.,0.]])
        self.kfz.H=np.array([[1.,0.]])
        self.kfx.R=np.array( [[ (2/image.shape[1])**2 ]] )
        self.kfx.update(x_update)
        self.kfy.R=np.array( [[ (2/image.shape[0])**2 ]] )
        self.kfy.update(y_update)
        #assume 5 cm resolution for depth
        if self.last_depth==0:
            #guess from height
            z_guess=image.shape[0]*focal_length*self.height/box[3]
            z_guess_unc=image.shape[0]*focal_length*self.height_unc/box[3]
            self.kfz.R=np.array( [[ z_guess_unc**2 ]])
            self.kfz.update(z_guess)
            logger.warning("Detection with zero depth!!, inferred {}+-{} from height".format(z_guess,z_guess_unc))
        else:
            self.kfz.R=np.array( [[ 0.05**2]])
            self.kfz.update(self.last_depth)
            height_update=self.kfz.x[0][0]*box[3]/(image.shape[0]*focal_length)
            #dh=h(dz/z+db/b)
            #dz is perhaps 10 percent of z
            #db is perhaps 3 pixels
            height_update_unc=height_update*np.sqrt(1e-2+(3/box[3])**2)
            w1=1/self.height_unc**2
            w2=1/height_update_unc**2
            self.height=(self.height*w2+height_update*w1)/(w1+w2)
            self.height_unc=min(self.height_unc,height_update_unc)
        if include_subimage:
            my_bbox=det["bbox_array"]
            my_bbox=[int(my_bbox[0]*self.shape[1]),int(my_bbox[1]*self.shape[1]),
                    int(my_bbox[2]*self.shape[0]),int(my_bbox[3]*self.shape[0])]
            self.subimage=image[my_bbox[2]:my_bbox[3],my_bbox[0]:my_bbox[1],:]
            if self.subimage.shape[0]==0 or self.subimage.shape[1]==0:
                self.subimage=[]
            #people can bend over, so I don't want it to be a normal dist improvement

#        self.kfx.H=np.array([[1.,1.]])
#        self.kfy.H=np.array([[1.,1.]])
#        self.kfz.H=np.array([[1.,1.]])
#        self.kfx.R=np.array( [[ (2/image.shape[1])**2,0 ],[0,xy_onesigma**2]])
#        self.kfx.update(np.array([[x_update,0]]))
#        self.kfy.R=np.array( [[ (2/image.shape[0])**2,0],[0,xy_onesigma**2]])
#        self.kfy.update(np.array([[y_update,0]]))
        #assume 5 cm resolution for depth
#        self.kfz.R=np.array( [[ 0.05**2,0 ],[0,velocity_onesigma]])
#        self.kfz.update(np.array([[self.last_depth,0]]))




        self.frames_without_detection=0
        self.frames_with_detection+=1
        self.info="DETECTED"

    def init_with_detection(self,image,det,include_subimage=False):
        self.shape=image.shape
        self.update_with_detection(det,image,include_subimage)

    def get_centroid(self):
        return (int(self.kfx.x[0][0]*self.shape[1]),int(self.kfy.x[0][0]*self.shape[1]))

    def get_det_bbox(self,det):
        x=int(0.5*(det["bbox_array"][0]+det["bbox_array"][1])*self.shape[1])
        y=int(0.5*(det["bbox_array"][2]+det["bbox_array"][3])*self.shape[0])
        w=int((det["bbox_array"][1]-det["bbox_array"][0])*self.shape[1])
        h=int((det["bbox_array"][3]-det["bbox_array"][2])*self.shape[0])
        return (x,y,w,h)

    def get_bestguess_bbox(self):
        centroid=self.get_centroid()
        return [centroid[0],centroid[1],self.last_det_bbox[2],self.last_det_bbox[3]]

    def get_bestguess_bbox_unscaled(self):
        [a,b,c,d]=self.get_bestguess_bbox()
        return [a/self.shape[1],b/self.shape[0],c/self.shape[1],d/self.shape[0]]

    def get_center(self):
        return [ self.kfx.x[0][0],self.kfy.x[0][0],self.kfz.x[0][0] ]

    def get_center_uncertainty(self):
        return [ np.sqrt(self.kfx.P[0][0]),np.sqrt(self.kfy.P[0][0]),np.sqrt(self.kfz.P[0][0]) ]

    def get_velocity(self):
        return [ self.kfx.x[1][0],self.kfy.x[1][0],self.kfz.x[1][0] ]



class TrackerGyrusNoCV(ThreadedGyrus):
    def __init__(self,broker,include_subimages=False):
        self.motion_corrector=MotionCorrection()
        self.offset_uncertainty=0.1 #fractional uncertainty on an offset
        self.include_subimages=include_subimages

        self.trackers=[]
        self.last_image_timestamp=0
        #conditions to remove tracker
        self.max_frames_without_detection=15 #0.5 second
        self.max_frames_offscreen=90 #3 seconds
        self.new_track_min_confidence=0.8
        super().__init__(broker)
        #for timing
        self.report_spf_count=30
        self.spf_count=0
        self.spf_sum_time=0

        #for size ineference
        self.object_height_table={
            "person": [1.5,0.5],
            "face": [0.2,0.1]
            }
        self.object_width_table={
            "person": [0.5,0.5],
            "face": [0.2,0.1]
            }



    def get_keys(self):
        return ["image","rotation_vector"]

    def get_name(self):
        return "TrackerGyrus"

    def getTracks(self):
        ret=[]
        for tracker in self.trackers:
            mess={}
            bb=tracker.get_bestguess_bbox_unscaled()
            mess["bbox_array"]=[ bb[0]-bb[2]/2,bb[0]+bb[2]/2,bb[1]-bb[3]/2,bb[1]+bb[3]/2]
            mess["center"]=tracker.get_center()
            mess["center_uncertainty"]=tracker.get_center_uncertainty()
            mess["velocity"]=tracker.get_velocity()
            mess["missed_frames"]=tracker.frames_without_detection
            mess["seen_frames"]=tracker.frames_with_detection
            mess["height"]=tracker.height
            mess["label"]=tracker.last_label
            mess["info"]=tracker.info
            mess["id"]=tracker.id
            if self.include_subimages:
                mess["subimage"]=tracker.subimage
            ret.append(mess)
        return ret

    def update_trackers(self,image,image_timestamp,offset_x,offset_y):
        if self.last_image_timestamp==0:
            self.last_image_timestamp=image_timestamp
            return
        dt=image_timestamp-self.last_image_timestamp

        for tracker in self.trackers:
            tracker.update_kf_from_time(dt)
            total_offset=offset_x+offset_y
            tracker.kfx.x[0]+=offset_x
            tracker.kfx.P[0][0]+=(total_offset*self.offset_uncertainty)**2
            tracker.kfy.x[0]+=offset_y
            tracker.kfy.P[0][0]+=(total_offset*self.offset_uncertainty)**2
            if 0<tracker.kfx.x[0]<1.0 and 0<tracker.kfy.x[0]<1.0:
                if tracker.frames_without_detection>min(self.max_frames_without_detection,tracker.frames_with_detection):
                    tracker.info="DISAPPEARED"
                    logger.warning("Track {} disappeared".format(id_to_name(tracker.id)))
                    tracker.defunct=True
            else:
                if tracker.kfx.x[0]<0:
                    tracker.info="EXIT LEFT"
                elif tracker.kfx.x[0]>1:
                    tracker.info="EXIT RIGHT"
                elif tracker.kfy.x[0]<0:
                    tracker.info="EXIT TOP"
                elif tracker.kfy.x[0]>1:
                    tracker.info="EXIT BOTTOM"
                if tracker.frames_without_detection>min(self.max_frames_offscreen,tracker.frames_with_detection):
                    tracker.defunct=True
            tracker.frames_without_detection+=1
        self.last_image_timestamp=image_timestamp

    def drop_dead_tracks(self):
        dead_trackers=[]
        for tracker in self.trackers:
            if tracker.defunct:
                dead_trackers.append(tracker)

        for tracker in dead_trackers:
            print("dropping {} with missed frames {}".format(tracker.last_label,tracker.frames_without_detection))
            self.trackers.remove(tracker)




    def get_match_score(self,tracker,det,imageshape):
        #so what's important?  Distance match, size match, and label match
        #metric: if you are halfway across the screen and the wrong label, let's call that a mismatch, even if same size
        #that's a distance of like 200 pixels.
        wrong_label_cost=0.3
        dist_cost_weight=0.7/200
        size_cost_weight=0.2/200
        label_cost=0
        if tracker.last_label!=det["label"]:
            label_cost=wrong_label_cost
        det_bbox=tracker.get_det_bbox(det)
        tracker_bbox=tracker.get_bestguess_bbox()
        dist_cost=dist_cost_weight*np.sqrt( (det_bbox[0]-tracker_bbox[0])**2+(det_bbox[1]-tracker_bbox[1])**2)
        size_cost=size_cost_weight*np.sqrt( (det_bbox[2]-tracker_bbox[2])**2+(det_bbox[3]-tracker_bbox[3])**2)
        return label_cost+dist_cost+size_cost

    def initialize_tracker(self,image,det):
        new_track=TrackerGyrusTrackedObject()
        if det["label"] in self.object_height_table:
            new_track.height=self.object_height_table[det["label"]][0]
            new_track.height_unc=self.object_height_table[det["label"]][0]
        new_track.init_with_detection(image,det,self.include_subimages)
        self.trackers.append(new_track)

    def match_trackers_to_detections(self,dets,image):
          #figure out best batches between the two
        cost_matrix=np.zeros( [len(dets),len(self.trackers)])
        for i in range(len(dets)):
            for j in range(len(self.trackers)):
                det=dets[i]
                tracker=self.trackers[j]
                cost_matrix[i,j]=self.get_match_score(tracker,det,image.shape)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        leftover_dets=np.setdiff1d(np.arange(len(dets)),row_ind)
        leftover_trackers=np.setdiff1d(np.arange(len(self.trackers)),col_ind)
        #logger.debug("Out of {} detections and {} tracks".format(len(dets),len(self.trackers)))
        #logger.debug("I've match up {} detections and tracks ".format(len(row_ind)))
        #logger.debug("leaving {} new detections and {} unsupported tracks".format(len(leftover_dets),len(leftover_trackers)))
        #deal with matches
        self.max_assignment_cost=1.0
        for i in range(len(row_ind)):
            if cost_matrix[row_ind[i],col_ind[i]]>self.max_assignment_cost:
                np.append(leftover_dets,row_ind[i])
                np.append(leftover_trackers,col_ind[i])
            else:
                self.trackers[col_ind[i]].update_with_detection(dets[row_ind[i]],image,include_subimage=self.include_subimages)
        #deal with leftover detections
        for i in range(len(leftover_dets)):
            if dets[leftover_dets[i]]["confidence"]>self.new_track_min_confidence:
                self.initialize_tracker(image,dets[leftover_dets[i]])
            else:
                logger.debug("Ignoring low confidence detection {} ({})".format(dets[leftover_dets[i]]["label"],dets[leftover_dets[i]]["confidence"]))
        #Do I need to deal with leftover tracks?  Maybe not, maybe deweight them TODO
        for i in range(len(leftover_trackers)):
            tracker=self.trackers[leftover_trackers[i]]
            if tracker.last_success:
                logger.debug("I have a tracker success but no detection for {}".format(id_to_name(tracker.id)))
            else:
                logger.debug("I have neither a tracker nor a detection for {}".format(id_to_name(tracker.id)))


    def read_message(self,message):
        self.motion_corrector.read_message(message)

        if "image" in message and "tracks" not in message:
            #logger.debug("Image Recieved")
            start_time=time.time()

            #handle heading correction
            offset_x,offset_y=self.motion_corrector.get_offset_and_update(message["image_timestamp"])
            #if abs(offset_y)>0.02:
            #    logger.info("offset y {}".format(offset_y))

            self.update_trackers(message["image"],message["image_timestamp"],offset_x,offset_y)

            #don't bother with detections if in a saccade, they're probably not right even if there
            if "detections" in message:
                #logger.debug("handling detections")
                self.match_trackers_to_detections(message["detections"],message["image"])

            message_out={"tracks": self.getTracks(),"timestamp": time.time(),"image_timestamp": message["image_timestamp"],"offset": [offset_x,offset_y]}
            self.broker.publish(message_out,["tracks"])
            self.drop_dead_tracks()

            #timing
            self.spf_count+=1
            self.spf_sum_time+=time.time()-start_time
            if self.spf_count>=self.report_spf_count:
                #logger.info("Tracking time fer frame {} ms".format(1000*self.spf_sum_time/self.spf_count))
                #logger.info("Number of Tracked obects now {}".format(len(self.trackers)))
                self.spf_sum_time=0
                self.spf_count=0
                for tracker in self.trackers:
                    ...
                    #print("Track: {} is a {} ".format(id_to_name(tracker.id),tracker.last_label))
                    #print("Location: {}".format(tracker.get_center()))
                    #print("Velocity: {}".format(tracker.get_velocity()))
