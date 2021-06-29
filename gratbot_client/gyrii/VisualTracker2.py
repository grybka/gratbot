#
from Gyrus import ThreadedGyrus
import time
import numpy as np
from scipy.linalg import block_diag
import torch
import cv2 as cv
import uuid
from gyrii.underpinnings.id_to_name import id_to_name
from gyrii.underpinnings.GratbotLogger import gprint
#from gyrii.underpinnings.Vodometer import Vodometer,VodometerException
from gyrii.underpinnings.BayesianArray import BayesianArray
from gyrii.underpinnings.ConvolutionalVisualOdometer import ConvolutionalVisualOdometer
from scipy.optimize import linear_sum_assignment
from filterpy.common import kinematic_kf,kinematic_state_transition,Q_discrete_white_noise

class VTTrackedObject3:
    """A Tracked object with a velocity kalman filter"""
    def __init__(self,startx,endx,starty,endy,label):
        #self.last_update=[startx,endx,starty,endy]
        self.label=label
        self.last_time_update=time.time()
        self.xy_kf=kinematic_kf(2,1,1/30.0) #I'll have to correct the state transition matrix each step
        self.wh_kf=kinematic_kf(2,1,1/30.0)
        #hardcode everything to start with P of 10 percent
        x,y,w,h=self.sxsyexeytoxywh(startx,starty,endx,endy)
        self.xy_kf.P=1e6*np.eye(4)
        self.wh_kf.P=1e6*np.eye(4)
        self.xy_kf.update( [x,y])
        self.wh_kf.update( [w,h])
        p=0.1*max(w,h)
        self.xy_kf.P=p*p*np.eye(4)
        self.wh_kf.P=p*p*np.eye(4)
        #TODO should I be adaptive in my measurement noise?
        self.xy_kf.R*=10
        self.wh_kf.R*=10

        #self.xy_q_variance=20000.0
        self.xy_q_variance=40000.0
        self.wh_q_variance=10.0
        self.info_vector=np.array([0.5*(startx+endx),0.5*(starty+endy),endx-startx,endy-starty,0,0]) #centerx,centery, xwidth,ywidth, vx, vy
        self.dt_threshhold=0.4
        self.label=label

        self.missed_frames=0
        self.last_frame_missed=False
        self.seen_frames=0
        self.id=uuid.uuid1()

    def apply_offset(self,xoffset,yoffset):
        self.xy_kf.x[0]-=xoffset
        self.xy_kf.x[2]-=yoffset

    def sxsyexeytoxywh(self,sx,sy,ex,ey):
        return 0.5*(sx+ex),0.5*(sy+ey),ex-sx,ey-sy

    def predict_to_time(self,timestamp):
        dt=timestamp-self.last_time_update
        if dt<self.dt_threshhold:
            return #just don't bother for too short a time
        self.xy_kf.F=block_diag(*([kinematic_state_transition(1,dt)]*2))
        self.xy_kf.Q=Q_discrete_white_noise(2,dt=dt,var=self.xy_q_variance,block_size=2)
        self.wh_kf.F=block_diag(*([kinematic_state_transition(1,dt)]*2))
        self.wh_kf.Q=Q_discrete_white_noise(2,dt=dt,var=self.wh_q_variance,block_size=2)
        self.xy_kf.predict()
        self.wh_kf.predict()
        self.last_time_update=timestamp

    def update(self,vision_object):
        self.missed_frames=0
        self.seen_frames+=1
        startx=vision_object["startx"]
        endx=vision_object["endx"]
        starty=vision_object["starty"]
        endy=vision_object["endy"]
        label=vision_object["label"]
        x,y,w,h=self.sxsyexeytoxywh(startx,starty,endx,endy)
        self.xy_kf.update( [x,y])
        self.wh_kf.update( [w,h])

    def get_predictions(self): #return startx,endx,starty,endy
        return self.xy_kf.x[0]-0.5*self.wh_kf.x[0],self.xy_kf.x[0]+0.5*self.wh_kf.x[0],self.xy_kf.x[2]-0.5*self.wh_kf.x[2],self.xy_kf.x[2]+0.5*self.wh_kf.x[2]

    def get_uncs(self):
        dx,dy,dw,dh=self.get_dxdydwdh()
        return np.sqrt(dx*dx+dw*dw),np.sqrt(dx*dx+dw*dw),np.sqrt(dy*dy+dh*dh),np.sqrt(dy*dy+dh*dh)


    def get_xywh(self):
        return float(self.xy_kf.x[0]),float(self.xy_kf.x[2]),float(self.wh_kf.x[0]),float(self.wh_kf.x[2])

    def get_vxvyvwvh(self):
        return float(self.xy_kf.x[1]),float(self.xy_kf.x[3]),float(self.wh_kf.x[1]),float(self.wh_kf.x[3])

    def get_dxdydwdh(self):
        return float(np.sqrt(self.xy_kf.P[0][0])),float(np.sqrt(self.xy_kf.P[2][2])),float(np.sqrt(self.wh_kf.P[0][0])),float(np.sqrt(self.wh_kf.P[2][2]))

    def get_expanded_predictions(self): #return startx,endx,starty,endy
        sx,ex,sy,ey=self.get_predictions()
        usx,uex,usy,uey=self.get_uncs()
        return sx-usx,ex+uex,sy-usy,ey+uey

    def get_bounding_box_overlap(self,bbvector):
        myvector=self.get_expanded_predictions()
        overlap_box=[
            max(bbvector[0],myvector[0]),
            min(bbvector[1],myvector[1]),
            max(bbvector[2],myvector[2]),
            min(bbvector[3],myvector[3])]
        min_area=min( (bbvector[1]-bbvector[0])*(bbvector[3]-bbvector[2]),
(myvector[1]-myvector[0])*(myvector[3]-myvector[2]))
        overlap_area=(overlap_box[1]-overlap_box[0])*(overlap_box[3]-overlap_box[2])
        return overlap_area/min_area

class VTTrackedObject2:
    """A Tracked object that doesn't have a velocity of its own"""
    def __init__(self,startx,endx,starty,endy,label):
        self.label=label
        self.last_update=[startx,endx,starty,endy]
        self.missed_frames=0
        self.last_frame_missed=False
        self.seen_frames=0
        self.id=uuid.uuid1()
        self.measurement_cov=100
        self.noise_cov=10
        cov=np.array([[self.measurement_cov,0],[0,self.measurement_cov]])
        self.x=BayesianArray(np.array([startx,endx]),cov.copy())
        self.y=BayesianArray(np.array([starty,endy]),cov.copy())

    #def update(self,startx,endx,starty,endy,label):
    def update(self,vision_object):
        startx=vision_object["startx"]
        endx=vision_object["endx"]
        starty=vision_object["starty"]
        endy=vision_object["endy"]
        label=vision_object["label"]
        cov=np.array([[self.measurement_cov,0],[0,self.measurement_cov]])
        self.x=self.x.updated(BayesianArray(np.array([startx,endx]),cov.copy()))
        self.y=self.y.updated(BayesianArray(np.array([starty,endy]),cov.copy()))
        self.label=label
        self.missed_frames=0
        self.seen_frames+=1
        self.last_frame_missed=False

    def apply_offset(self,xoffset,yoffset,xoffset_unc,yoffset_unc):
        self.x.vals+=np.array([xoffset,xoffset])
        self.y.vals+=np.array([yoffset,yoffset])
        self.x.covariance[0][0]+=xoffset_unc*xoffset_unc
        self.x.covariance[1][1]+=xoffset_unc*xoffset_unc
        self.x.covariance[0][1]-=xoffset_unc*xoffset_unc
        self.x.covariance[1][0]-=xoffset_unc*xoffset_unc
        self.y.covariance[0][0]+=yoffset_unc*yoffset_unc
        self.y.covariance[1][1]+=yoffset_unc*yoffset_unc
        self.y.covariance[0][1]-=yoffset_unc*yoffset_unc
        self.y.covariance[1][0]-=yoffset_unc*yoffset_unc

    def predict(self,dt):
        self.add_covariance(self.noise_cov*dt)

    def add_covariance(self,cov):
        self.x.covariance[0][0]+=cov
        self.x.covariance[1][1]+=cov
        self.y.covariance[0][0]+=cov
        self.y.covariance[1][1]+=cov

    def get_predictions(self): #return startx,endx,starty,endy
        return self.x.vals[0],self.x.vals[1],self.y.vals[0],self.y.vals[1],

    def get_uncs(self):
        return np.sqrt(self.x.covariance[0][0]),np.sqrt(self.x.covariance[1][1]),np.sqrt(self.y.covariance[0][0]),np.sqrt(self.y.covariance[1][1])

    def get_expanded_predictions(self): #return startx,endx,starty,endy
        sx,ex,sy,ey=self.get_predictions()
        usx,uex,usy,uey=self.get_uncs()
        return sx-usx,ex+uex,sy-usy,ey+uey

    def get_xywh(self):
        sx,ex,sy,ey=self.get_predictions()
        return 0.5*(sx+ex),0.5*(sy+ey),ex-sx,ey-sy


    def get_dxdydwdh(self):
        center_array=np.array([0.5,0.5])
        wh_array=np.array([-1.0,1.0])
        dx=np.sqrt(np.dot(center_array,np.dot(self.x.covariance,center_array)))
        dy=np.sqrt(np.dot(center_array,np.dot(self.y.covariance,center_array)))
        dw=np.sqrt(np.dot(wh_array,np.dot(self.x.covariance,wh_array)))
        dh=np.sqrt(np.dot(wh_array,np.dot(self.y.covariance,wh_array)))
        return [dx,dy,dw,dh]

    def get_bounding_box_overlap(self,bbvector):
        myvector=self.get_expanded_predictions()
        overlap_box=[
            max(bbvector[0],myvector[0]),
            min(bbvector[1],myvector[1]),
            max(bbvector[2],myvector[2]),
            min(bbvector[3],myvector[3])]
        min_area=min( (bbvector[1]-bbvector[0])*(bbvector[3]-bbvector[2]),
(myvector[1]-myvector[0])*(myvector[3]-myvector[2]))
        overlap_area=(overlap_box[1]-overlap_box[0])*(overlap_box[3]-overlap_box[2])
        return overlap_area/min_area


def get_rot_matrix3(theta):
    return np.array([[np.cos(theta),np.sin(theta),0],
                     [-np.sin(theta),np.cos(theta),0],
                     [0,0,1]])

class VisualTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,display,show_detections=True):
        self.show_detections=show_detections
        self.display=display
        self.objects_to_track=["chair","person","sports ball","stop sign"]
        #self.objects_to_track=["sports ball","stop sign"]
        self.in_saccade=True
        self.clear_frames_before=0
        self.last_pose_as_obj=None

        self.tracked_objects=[]


        #self.minimum_track_assignment_threshhold=-0.2
        #self.minimum_track_assignment_threshhold=-0.1
        self.minimum_track_assignment_threshhold=-0.25

        #Tagging info
        self.tagger_model= torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.tagger_classes=self.tagger_model.module.names if hasattr(self.tagger_model,'module') else self.tagger_model.names

        #Odometer info
        self.vodometer=ConvolutionalVisualOdometer(margin=40)

        super().__init__(broker)

        #for profiling
        #self.n_iter_report=30
        self.n_iter_report=-1
        self.on_iter=0

        self.tagging_time_sum=0
        self.vodometer_time_sum=0

        #for testing odometer
        self.marker_x=320
        self.marker_y=240
        self.startup_time=time.time()



    def get_keys(self):
        return [ "camera_frame","drive/motors_active","latest_pose" ]

    def get_name(self):
        return "VisualTrackerGyrus"

    def read_message(self,message):
        self.in_saccade=False
        if "latest_pose" in message:
            self.last_pose_as_obj=message["latest_pose"]
#        if "drive/motors_active" in message:
#            motors_active=message["drive/motors_active"][0:3]
#            if motors_active!=[0,0,0]:
#                self.in_saccade=True
#            else:
#                self.in_saccade=False
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                return
            if self.in_saccade==True:
                return
            retval=self.update_with_frame(message["camera_frame"],message["timestamp"])
            if self.show_detections and retval:
                self.draw_bboxes(message["camera_frame"])

    def get_cost(self,tracked_object,vision_object):
        #this is bounding box overlap
        vision_bbox=[vision_object["startx"],vision_object["endx"],vision_object["starty"],vision_object["endy"]]
        overlap=tracked_object.get_bounding_box_overlap(vision_bbox)
        cost=-0.5*overlap
        #this is absolute distance scaled by 1/4 of screen
        x,y,w,h=tracked_object.get_xywh()
        dx=(x-0.5*(vision_object["startx"]+vision_object["endx"]))
        dy=(y-0.5*(vision_object["starty"]+vision_object["endy"]))
        dist=np.sqrt(dx*dx+dy*dy)
        sigma=640/4
        cost-=0.5*np.exp(dist*dist/(2*sigma*sigma))


        #wrong_class_penalty=0.75
        wrong_class_penalty=10.0
        if vision_object["label"]!=tracked_object.label:
            cost+=wrong_class_penalty
        return cost

    def apply_offset(self,xoffset,yoffset,timestamp):
        if xoffset is None or yoffset is None:
            return
        #gprint("n tracked objects {}".format(len(self.tracked_objects)))
        for obj in self.tracked_objects:
            obj.apply_offset(xoffset,yoffset)
            #gprint("{}".format(obj.xy_kf.x[1]))
        self.marker_x-=xoffset
        self.marker_y-=yoffset
        #here I figure out what this means in terms of my own pose
        if self.last_pose_as_obj is not None:
            #TODO re do this with the assumption that this is a velocity measurement
            starting_pose=BayesianArray.from_object(self.last_pose_as_obj)
            x,cov=self.vodometer.get_pose_change(xoffset)
            large_unc=100.0
            offset_egoframe=BayesianArray( np.array([x[1],0,x[0]]),np.array([[cov[1][1],0,cov[1][0]],[0,large_unc,0],[cov[0][1],0,cov[0][0]]]))
            rot_matrix=get_rot_matrix3(starting_pose.vals[2])
            #gprint("for xoffset of {}, {}".format(xoffset,yoffset))
            offset=offset_egoframe.applymatrix(rot_matrix)
            #gprint("offset is {}".format(offset.pretty_str()))
            new_pose=starting_pose+offset
            #gprint("new pose object is {}".format(new_pose.to_object()))
            self.broker.publish({"pose_measurement": new_pose.to_object(),"timestamp": timestamp,"notes": "visual"},"pose_measurement")

            #TODO use this for velocity measurement

            #vel_measurement=offset
            #vel_measurement.vals/=self.vodometer.velocity_timebase
            #self.broker.publish({"velocity_measurement": (new_pose.to_object(),"timestamp": timestamp,"notes": "visual"},"pose_measurement")

    def update_with_frame(self,frame,timestamp):
        #odometry here
        start_time=time.time()
        xoffset,yoffset=self.vodometer.update(frame,timestamp)
        if xoffset is not None:
            self.broker.publish({"video_offset": [xoffset,yoffset,self.vodometer.x_velocity],"timestamp": timestamp,"notes": "visual"},"video_offset")
#        if xoffset!=0 or yoffset!=0:
#            gprint("xoffset {} yoffset {}".format(xoffset,yoffset))
        self.apply_offset(xoffset,yoffset,timestamp)
        self.vodometer_time_sum+=time.time()-start_time

        #TODO this function will matter if I kalman filter on objects
        self.update_tracked_objects_to_time(timestamp)
        tagged_objects=self.tag_objects(frame)
        self.broker.publish({"tagged_objects": tagged_objects,"timestamp": timestamp,"notes": "visual"},"tagged_objects")
        trackable_tagged_objects=list(filter( lambda x: x["label"] in self.objects_to_track, tagged_objects))
        #make my cost matrix to assign each seen object to one I'm tracking
        cost_matrix=np.zeros([len(self.tracked_objects),len(trackable_tagged_objects)])
        for i in range(len(self.tracked_objects)):
            for j in range(len(trackable_tagged_objects)):
                cost_matrix[i,j]=self.get_cost(self.tracked_objects[i],trackable_tagged_objects[j])
        tracked_ind,tagged_ind=linear_sum_assignment(cost_matrix)
        missing_track_inds=list(set(range(len(self.tracked_objects))).difference(set(tracked_ind)))
        missing_tag_inds=list(set(range(len(trackable_tagged_objects))).difference(set(tagged_ind)))
        #for each assigned pair, update track with new measurement if cost is below threshhold
        for i in range(len(tracked_ind)):
            if cost_matrix[tracked_ind[i],tagged_ind[i]]<self.minimum_track_assignment_threshhold:
                self.tracked_objects[tracked_ind[i]].update(trackable_tagged_objects[tagged_ind[i]])
            else:
                gprint("cost {} too high".format(cost_matrix[tracked_ind[i],tagged_ind[i]]))
                missing_track_inds.append(tracked_ind[i])
                missing_tag_inds.append(tagged_ind[i])
        #for each tag that has no track, create a new object
        if len(missing_tag_inds)>0:
            gprint("adding {} tracked objects".format(len(missing_tag_inds)))
        for i in missing_tag_inds:
                sx,ex,sy,ey=trackable_tagged_objects[i]["startx"],trackable_tagged_objects[i]["endx"],trackable_tagged_objects[i]["starty"],trackable_tagged_objects[i]["endy"]
                #new_track=VTTrackedObject2(sx,ex,sy,ey,trackable_tagged_objects[i]["label"])
                new_track=VTTrackedObject3(sx,ex,sy,ey,trackable_tagged_objects[i]["label"])
                self.tracked_objects.append(new_track)
        #for each track unpaired, if I -should- see it, then note that its missing
        for i in missing_track_inds:
            #gprint("missing object, missed frames {}".format(self.tracked_objects[i].missed_frames))
            self.tracked_objects[i].missed_frames+=1
            self.tracked_objects[i].last_frame_missed=True

        #remove tracks with too many missed frames
        max_missed_frames=30
        todrop=filter( lambda obj: (obj.missed_frames>max_missed_frames) or (obj.missed_frames>obj.seen_frames), self.tracked_objects)
        dropped_ids=[]
        for obj in todrop:
            #gprint("missed_frames {}".format(obj.missed_frames))
            dropped_ids.append(obj.id)
            gprint("dropping {} objects".format(obj.id))
            self.tracked_objects.remove(obj)

        self.on_iter+=1
        if self.n_iter_report>0 and self.on_iter>self.n_iter_report:
            tagging_time=self.tagging_time_sum/self.on_iter
            gprint("Average Tagging Time {} ms".format(tagging_time*1000))
            vodometer_time=self.vodometer_time_sum/self.on_iter
            gprint("Average Vodometer Time {} ms".format(vodometer_time*1000))
            self.tagging_time_sum=0
            self.vodometer_time_sum=0
            self.on_iter=0

        if len(self.tracked_objects)>0 or len(dropped_ids)>0:
            mymessage={"timestamp": timestamp,"visual_tracker_objects": self.serialize_tracks(), "visual_tracker_dropped_objects": dropped_ids}
            #, "pose_used": self.last_pose_as_obj }
            self.broker.publish(mymessage,"visual_tracker_objects")

        return True



    def update_tracked_objects_to_time(self,timestamp):
        for obj in self.tracked_objects:
            obj.predict_to_time(timestamp)

    def tag_objects(self,frame):
        start_time=time.time()
        imgs=[frame]
        results = self.tagger_model(imgs)
        video_objects=[]
        for r in results.xyxy[0]:
            video_objects.append({
                "confidence": r[4].item(),
                "label": self.tagger_classes[int(r[5].item())],
                "startx": r[0].item(),
                "starty": r[1].item(),
                "endx": r[2].item(),
                "endy": r[3].item()
            })
        self.tagging_time_sum+=time.time()-start_time
        return video_objects


    def serialize_tracks(self):
        ret=[]
        for obj in self.tracked_objects:
            if obj.missed_frames==0: #don't report on objects I didn't actually tag
                item={}
                item["label"]=obj.label
                item["id"]=str(obj.id)
                #item["last_update"]=obj.last_update
                item["xywh"]=obj.get_xywh()
                item["vxvyvwvh"]=obj.get_vxvyvwvh()
                item["dxdydwdh"]=obj.get_dxdydwdh() #START HERE
                ret.append(item)
        return ret
    def draw_bboxes(self,video_frame):

        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        try:
            ret_frame=video_frame.copy()

            for obj in self.tracked_objects:
                #if obj.last_frame_missed:
                #    continue
                sx,ex,sy,ey=obj.get_predictions()
                usx,uex,usy,uey=obj.get_uncs()
                cv.rectangle(ret_frame,(int(sx-usx),int(sy-usy)),(int(ex+uex),int(ey+uey)),(0,255,0),2)
                text = "{} {}".format("Tracked",obj.label)
                Y = int(sy - 10 if sy - 10 > 10 else sy + 10)
                cv.putText(ret_frame, text, (int(sx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)

        except:
            gprint("where is nan {} {} {} {}, {} {} {} {}".format( sx,ex,sy,ey, usx,uex,usy,uey))
        #gprint("marker x {} marker y {}".format(self.marker_x,self.marker_y))
        cv.drawMarker(ret_frame,(self.marker_x,self.marker_y),(255,0,0))
        self.display.update_image("visual_tracker",ret_frame)
