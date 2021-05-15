from Gyrus import ThreadedGyrus
import numpy as np
import time
import cv2 as cv
import sys
sys.path.append('../yolov5_tool')
from yolov5_tool import yolov5_tool
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.optimize import linear_sum_assignment
import uuid
from gyrii.underpinnings.BayesianArray import BayesianArray
from gyrii.underpinnings.id_to_name import id_to_name
from gyrii.underpinnings.GratbotLogger import gprint
from gyrii.underpinnings.Vodometer import Vodometer,VodometerException



class ObjectTagger():
    def __init__(self):
        self.model_file="C:/Users/grybk/projects/yolov5/yolov5/weights/yolov5m.pt"
        self.video_width=640
        self.video_height=480
        self.yv5model=None
        self.yv5model=yolov5_tool()



    def tag_objects(self,video_frame):
        if self.yv5model.initialized==False:
            gprint("initializing yolov5 model")
            self.yv5model.initialize(self.model_file)
            gprint("initialized")
        #logging.info("detecting")
        try:
            video_objects=self.yv5model.detect(video_frame)
        except Exception as e:
            print("video frame is {}".format(video_frame))
            print("Exception: {}".format(e))
            print("Unexpected error:", sys.exc_info()[0])
        #logging.info("detected")
        return video_objects

    def read_message(self,message):
        if "camera_frame" in message:
            if message["timestamp"]<self.clear_frames_before:
                return
            video_objects=self.tag_objects(message["camera_frame"])
            #gprint("number of video objects: {}".format(len(video_objects)))
            mymessage={"timestamp":message["timestamp"],"video_object_tags": video_objects}
            self.broker.publish(mymessage,"video_object_tags")
            if self.show_detections:
                self.display_tags(message["camera_frame"],video_objects)
            self.clear_frames_before=time.time()

    def display_tags(self,video_frame,video_objects):
        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        return_frame=video_frame.copy()
        self.yv5model.draw_object_bboxes(return_frame,video_objects)
        self.display.update_image("object_tagger",return_frame)


#    def get_obj_loc_width(self,face):
#        #converts the start,stop notation into an array of points, center and wh
#        centerx=(0.5*(face["startx"]+face["endx"])-self.video_width/2)
#        centery=(0.5*(face["starty"]+face["endy"])-self.video_height/2)
#        width=(-face["startx"]+face["endx"])
#        height=(-face["starty"]+face["endy"])
#        return np.array([centerx,centery]),np.array([width,height])

class VTTrackedObject:
    #def __init__(self,x,y,w,h,label):
    def __init__(self,startx,endx,starty,endy,label):
        self.label=label
        self.noise_var=100
        self.kfx = KalmanFilter(dim_x=3,dim_z=2) #startx,endx, xvel
        self.kfy = KalmanFilter(dim_x=3,dim_z=2) #startx,endx, xvel
        self.kfx.x=np.array([[startx, endx,0]]).T # initially at 0,0 with 0 velocity
        self.kfy.x=np.array([[starty, endy,0]]).T # initially at 0,0 with 0 velocity
        #transition matrix (x=x0+v*t)
        self.kfx.F=np.array([ [1.0,0.0,1.0],
                              [0.0,1.0,1.0],
                              [0.0,0.0,1.0] ])
        self.kfy.F=np.array([ [1.0,0.0,1.0],
                              [0.0,1.0,1.0],
                              [0.0,0.0,1.0] ])
        #measurement function (only position)
        self.kfx.H=np.array([[1.0,0.0,0.0],[0.0,1.0,0.0]])
        self.kfy.H=np.array([[1.0,0.0,0.0],[0.0,1.0,0.0]])

        #covariance matrix
        self.kfx.P *= 100. #I guess I figure this is big
        self.kfy.P *= 100. #I guess I figure this is big

        #Fundamental maesurement noise
        self.kfx.R =np.array([[100,0],
                              [0,100]])
        self.kfy.R =np.array([[100,0],
                              [0,100]])


        dt=1
        n=self.noise_var*dt
        self.kfx.Q=np.array([[n*dt,0   ,n*dt],
                             [0   ,n*dt,n*dt],
                             [n*dt,n*dt,n]])
        self.kfy.Q=np.array([[n*dt,0   ,n*dt],
                             [0   ,n*dt,n*dt],
                             [n*dt,n*dt,n]])
        #self.kfx.Q=Q_discrete_white_noise(dim=3,dt=1.0,var=1)
        #self.kfy.Q=Q_discrete_white_noise(dim=3,dt=1.0,var=1)

        self.last_update=[startx,endx,starty,endy]
        self.missed_frames=0
        self.seen_frames=0
        self.id=uuid.uuid1()
        #self.associated_map_object=None

    def predict(self,dt):
        self.kfx.F=np.array([ [1.0,0.0,dt],
                              [0.0,1.0,dt],
                              [0.0,0.0,1.0] ])
        self.kfy.F=np.array([ [1.0,0.0,dt],
                              [0.0,1.0,dt],
                              [0.0,0.0,1.0] ])
        n=self.noise_var*dt
        self.kfx.Q=np.array([[n*dt,0   ,n*dt],
                             [0   ,n*dt,n*dt],
                             [n*dt,n*dt,n]])
        self.kfy.Q=np.array([[n*dt,0   ,n*dt],
                             [0   ,n*dt,n*dt],
                             [n*dt,n*dt,n]])
        #self.kfx.Q=Q_discrete_white_noise(dim=3,dt=dt,var=self.noise_var)
        #self.kfy.Q=Q_discrete_white_noise(dim=3,dt=dt,var=self.noise_var)
        self.kfx.predict()
        self.kfy.predict()

    def apply_offset(self,xoffset,yoffset):
        self.kfx.x[0][0]+=xoffset
        self.kfx.x[1][0]+=xoffset
        self.kfx.y[0][0]+=yoffset
        self.kfx.y[1][0]+=yoffset

    def update(self,startx,endx,starty,endy,label):
        self.last_update=[startx,endx,starty,endy]

        #print("update tracked object at {} {} {} {} {}".format(x,y,w,h,label))
        self.kfx.update([startx,endx])
        self.kfy.update([starty,endy])
        self.label=label
        self.missed_frames=0
        self.seen_frames+=1

    def add_covariance(self,cov):
        gprint("P is {} before".format(self.kfx.P))
        self.kfx.P[0][0]+=cov
        self.kfx.P[1][1]+=cov
        self.kfy.P[0][0]+=cov
        self.kfy.P[1][1]+=cov
        gprint("P is {} after".format(self.kfx.P))


    def get_predictions(self): #return startx,endx,starty,endy
        return self.kfx.x[0],self.kfx.x[1],self.kfy.x[0],self.kfy.x[1]

    def get_expanded_predictions(self): #return startx,endx,starty,endy
        sx,ex,sy,ey=self.get_predictions()
        usx,uex,usy,uey=self.get_uncs()
        return sx-usx,ex+uex,sy-usy,ey+uey


    def get_uncs(self):
        return np.sqrt(self.kfx.P[0][0]),np.sqrt(self.kfx.P[1][1]),np.sqrt(self.kfy.P[0][0]),np.sqrt(self.kfy.P[1][1])

    def get_xywh(self):
        return [0.5*(self.kfx.x[0][0]+self.kfx.x[1][0]),0.5*(self.kfy.x[0][0]+self.kfy.x[1][0]),self.kfx.x[1][0]-self.kfx.x[0][0],self.kfy.x[1][0]-self.kfy.x[0][0]]

    def get_dxdydwdh(self):
        center_array=np.array([0.5,0.5])
        wh_array=np.array([-1.0,1.0])
        dx=np.sqrt(np.dot(center_array,np.dot(self.kfx.P[0:2,0:2],center_array)))
        dy=np.sqrt(np.dot(center_array,np.dot(self.kfy.P[0:2,0:2],center_array)))
        dw=np.sqrt(np.dot(wh_array,np.dot(self.kfx.P[0:2,0:2],wh_array)))
        dh=np.sqrt(np.dot(wh_array,np.dot(self.kfy.P[0:2,0:2],wh_array)))
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



class VTTrackedObject2:
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

    def update(self,startx,endx,starty,endy,label):
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

class VisualTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker,display,show_detections=True):
        self.show_detections=show_detections
        self.display=display
        self.object_tagger=ObjectTagger()
        self.tracked_objects=[]
        #self.id_names_hash=["Franz","Brunhilda","Steve","Persepolis","Legolas","Senator","Doublewide","Convolution","Beaucephalus","Microencephalus","Enchilada","Buttercup","Malferious","Ferrous","Titiana","Europa","Malpractice","Daedelus","Dad-elus","Cheesemonger","Burgertime","ForgetMeNot","Nevar4get","Allowance","Betrayal","Elvis"]
        #self.camera_hfov=48.8*(2*np.pi)/360 #from spec sheet
        self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        #self.object_heights={ "stop sign": [0.081,0.005], "sports ball": [0.115,0.01]}
        self.clear_frames_before=0
        #self.objects_to_track=["chair"]
        self.objects_to_track=["chair","person","sports ball","stop sign"]
        self.last_timestamp=time.time()
        self.vodometer=Vodometer()
        self.max_tracked_objects=1 # 3
        self.minimum_track_assignment_threshhold=-0.2
        self.in_saccade=True
        self.last_pose_as_obj=None
        #self.minimum_track_assignment_threshhold=1e9
        super().__init__(broker)
        self.time_sum=0
        self.sample_sum=0
        self.n_sample=10

    def get_keys(self):
        return [ "camera_frame","drive/motors_active","latest_pose" ]

    def get_name(self):
        return "VisualTrackerGyrus"

    def apply_offset(self,xoffset,yoffset,xoffset_unc,yoffset_unc):
        #gprint("apply offset {}+-{} {}+-{}".format(xoffset,xoffset_unc,yoffset,yoffset_unc))

        #if abs(xoffset)>10:
        #     gprint("apply offset {}+-{} {}+-{}".format(xoffset,xoffset_unc,yoffset,yoffset_unc))
        #    gprint("apply offset {} {}".format(xoffset,yoffset))

        for obj in self.tracked_objects:
            obj.apply_offset(xoffset,yoffset,xoffset_unc,yoffset_unc)

    def get_cost(self,tracked_object,vision_object):
        vision_bbox=[vision_object["startx"],vision_object["endx"],vision_object["starty"],vision_object["endy"]]
        overlap=tracked_object.get_bounding_box_overlap(vision_bbox)
        #GOT THIS FAR
        #gprint("vision box {}".format(vision_bbox))
        #gprint("object box {}".format(tracked_object.get_predictions()))
        cost=-overlap
        wrong_class_penalty=0.4
        if vision_object["label"]!=tracked_object.label:
            cost+=wrong_class_penalty
        return cost

    def read_message(self,message):
        if "latest_pose" in message:
            self.last_pose_as_obj=message["latest_pose"]
        if "camera_frame" in message and self.last_pose_as_obj is not None:
            if message["timestamp"]<self.clear_frames_before:
                return
            if self.in_saccade==True:
                return

            start_time=time.time()
            retval=self.update(message["camera_frame"],message["timestamp"])
            self.time_sum+=time.time()-start_time
            self.sample_sum+=1
            if self.sample_sum>=self.n_sample:
                gprint("average tagger time {} ms".format(1000*self.time_sum/self.sample_sum))
                self.sample_sum=0
                self.time_sum=0

            if self.show_detections and retval:
                self.display_tags(message["camera_frame"])
            self.clear_frames_before=time.time()
        if "drive/motors_active" in message:
            motors_active=message["drive/motors_active"][0:3]
            if motors_active!=[0,0,0]:
                self.in_saccade=True
            else:
                self.in_saccade=False

    def display_tags(self,frame):
        return_frame=frame.copy()
        return_frame=self.draw_bboxes(return_frame)
        #self.yv5model.draw_object_bboxes(return_frame,video_objects)
        self.display.update_image("visual_tracker",return_frame)

    def update(self,frame,timestamp):
        try:
            xoffset,yoffset,xoffset_unc,yoffset_unc=self.vodometer.get_offset_since_last(frame)
        except VodometerException as e:
            for obj in self.tracked_objects:
                obj.add_covariance(10)
            if e.message=="No Match": #if this is the case ,then I've lost all sense of motion
                #gprint("tracking lost")
                for obj in self.tracked_objects:
                    obj.add_covariance(100) #How do I pick this number?
            return False#No key points is a good sign tracking will be messed up
        #if the odometer is within 1 sigma, then don't jerk the camera around, just use uncertainty
        if xoffset!=0 and xoffset_unc/xoffset>1:
            xoffset=0
        if yoffset!=0 and yoffset_unc/yoffset>1:
            yoffset=0
        self.apply_offset(xoffset,yoffset,xoffset_unc,yoffset_unc)
        self.dt=timestamp-self.last_timestamp
        self.last_timestamp=timestamp
        minimum_track_assignment_threshhold=self.minimum_track_assignment_threshhold
        max_tracked_objects=self.max_tracked_objects
        vision_objects=self.object_tagger.tag_objects(frame)
        #pare this down to only tracked objects
        trackables=[]
        for x in vision_objects:
            if x["label"] in self.objects_to_track:
                trackables.append(x)
        vision_objects=trackables
        #gprint("{} vision objects".format(vision_objects))

        #gprint("{} vision objects {} tracked objects".format(len(vision_objects),len(self.tracked_objects)))
        unused_vision_objects=vision_objects
        for objind in range(len(self.tracked_objects)):
            the_costs=[]
            #predict where the object should be
            self.tracked_objects[objind].predict(self.dt)
            for visind in range(len(vision_objects)):
                the_costs.append(self.get_cost(self.tracked_objects[objind],unused_vision_objects[visind]))
            best_fit=-1
            if len(the_costs)>=1:
                best_fit=np.argmin(the_costs)
            if best_fit==-1 or the_costs[best_fit]>minimum_track_assignment_threshhold:
                self.tracked_objects[objind].missed_frames+=1
                self.tracked_objects[objind].last_frame_missed=True

                #gprint("poor fit to {}".format(self.tracked_objects[objind].label))
                #gprint("best fit is {}".format(best_fit))
                #gprint("cost is  {}".format(the_costs[best_fit]))
            else:
                #match for object
                sx,ex,sy,ey=unused_vision_objects[best_fit]["startx"],unused_vision_objects[best_fit]["endx"],unused_vision_objects[best_fit]["starty"],unused_vision_objects[best_fit]["endy"]
                #terr=sx-self.tracked_objects[objind].get_predictions()[0]
                #if abs(terr)>20:
                #    gprint("Track Error {} {}".format(sx-self.tracked_objects[objind].get_predictions()[0],sy-self.tracked_objects[objind].get_predictions()[2]))
                self.tracked_objects[objind].update(sx,ex,sy,ey,unused_vision_objects[best_fit]["label"])
                unused_vision_objects.remove(unused_vision_objects[best_fit])
                #gprint("matching {}".format(self.tracked_objects[objind].label))
        #any additional vision objects become potential tracks!
        if len(unused_vision_objects)>0 and len(self.tracked_objects)<=max_tracked_objects:
            #xy,wh=self.object_tagger.get_obj_loc_width(unused_vision_objects[0])
            sx,ex,sy,ey=unused_vision_objects[0]["startx"],unused_vision_objects[0]["endx"],unused_vision_objects[0]["starty"],unused_vision_objects[0]["endy"]
            #new_track=VTTrackedObject(sx,ex,sy,ey,unused_vision_objects[0]["label"])
            new_track=VTTrackedObject2(sx,ex,sy,ey,unused_vision_objects[0]["label"])
            #gprint("adding {} as potential track".format(new_track.label))
            self.tracked_objects.append(new_track)
            #####


        #remove tracks with too many missed frames
        max_missed_frames=10
        to_drop=[]
        dropped_ids=[]
        for obj in self.tracked_objects:
            if obj.missed_frames>=max_missed_frames:
                #gprint("Dropping {} because too many missed frames".format(obj.label))
                to_drop.append(obj)
            elif obj.missed_frames>obj.seen_frames:
                #gprint("Dropping {} because more often missed than seen {} and {}".format(obj.label,obj.missed_frames,obj.seen_frames))
                to_drop.append(obj)
        for obj in to_drop:
            if obj.id not in dropped_ids:
                dropped_ids.append(obj.id)
            self.tracked_objects.remove(obj)
        #tnames=[n.label for n in self.tracked_objects]
        #gprint("Tracked objects {}".format(tnames))

        #TODO go back and handle literal edge cases

        #make guesses about where these are
        mymessage={"timestamp": timestamp,"visual_tracker_objects": self.serialize_tracks(), "visual_tracker_dropped_objects": dropped_ids, "pose_used": self.last_pose_as_obj }
        #gprint("publishing vtracker objects")
        self.broker.publish(mymessage,"visual_tracker_objects")
        return True



    def draw_bboxes(self,video_frame):
        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        try:
            ret_frame=video_frame.copy()

            cv.drawKeypoints(ret_frame,self.vodometer.last_features_kp,ret_frame,color=(0,255,0),flags=0)
            for obj in self.tracked_objects:
                if obj.last_frame_missed:
                    continue
                #sxf,exf,syf,eyf=obj.get_extended_bounding_box()
                #sx=320+sxf*640
                #sy=240+syf*480
                #ex=320+exf*640
                #ey=240+eyf*480
                sx,ex,sy,ey=obj.get_predictions()
                usx,uex,usy,uey=obj.get_uncs()
                #gprint("{} {} {} {}".format(usx,uex,usy,uey))
                #print("drawing box at {} {} {} {}".format(sx,sy,ex,ey))
                #cv.rectangle(ret_frame,(int(sx),int(sy)),(int(ex),int(ey)),(0,255,0),2)
                #gprint("{} {} {} {}".format(usx,uex,usy,uey))
                cv.rectangle(ret_frame,(int(sx-usx),int(sy-usy)),(int(ex+uex),int(ey+uey)),(0,255,0),2)
                #text = "{} {}".format(obj.label,obj.missed_frames)
                #text = "{} {}".format(self.id_names_hash[hash(obj.id)%len(self.id_names_hash)],obj.missed_frames)
                #name="unknown"
                #if obj.associated_map_object is not None:
                #    name=id_to_name(obj.associated_map_object)
                text = "{} {}".format("Tracked",obj.label)
                Y = int(sy - 10 if sy - 10 > 10 else sy + 10)
                cv.putText(ret_frame, text, (int(sx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
        except:
            gprint("where is nan {} {} {} {}, {} {} {} {}".format( sx,ex,sy,ey, usx,uex,usy,uey))
        return ret_frame

    def serialize_tracks(self):
        ret=[]
        for obj in self.tracked_objects:
            if obj.missed_frames==0: #don't report on objects I didn't actually tag
                item={}
                item["label"]=obj.label
                item["id"]=str(obj.id)
                item["last_update"]=obj.last_update
                item["xywh"]=obj.get_xywh()
                item["dxdydwdh"]=obj.get_dxdydwdh() #START HERE
                ret.append(item)
        return ret

    def retrieve_object(self,id):
        for obj in self.tracked_objects:
            if obj.id==id:
                return obj
        gprint("{} not found".format(id))
        return None

    def predict_angle_from_track(self,track):
        #I assume here that my camera points straight ahead
        pred=track.kfx.x[0]
        dpred=np.sqrt(track.kfx.P[0][0])
        #print("pred {} dpred {}".format(pred,dpred))
        angle_guess=pred*self.camera_wfov
        dangle_guess=dpred*self.camera_wfov
        return angle_guess,dangle_guess

    def predict_dist_from_track(self,track):
        if track.label in self.object_heights:
            h=self.object_heights[track.label][0]
            dh=self.object_heights[track.label][1]
            pred=track.kfh.x[0]
            dpred=np.sqrt(track.kfh.P[0][0])
            dist_guess=h/np.tan(pred*self.camera_hfov)
            dist_guess_unc=dist_guess*np.sqrt( (dh/h)**2+ (dpred/pred)**2 )
            return dist_guess,dist_guess_unc
        else:
            return 5,100
