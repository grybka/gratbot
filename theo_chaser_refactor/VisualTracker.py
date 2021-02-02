from Theo_Chaser_Object_Tagger import Theo_Chaser_Object_Tagger
import numpy as np
import cv2 as cv
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.optimize import linear_sum_assignment
import uuid
from id_to_name import id_to_name


class VTTrackedObject:
    def __init__(self,x,y,w,h,label):
        print("new tracked object at {} {} {} {} {}".format(x,y,w,h,label))
        self.label=label
        self.kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        self.kfy = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        self.kfw = KalmanFilter(dim_x=1,dim_z=1) #xxpos,       xmeas
        self.kfh = KalmanFilter(dim_x=1,dim_z=1) #xxpos,       xmeas
        self.kfx.x=np.array([x, 0]) # initially at 0,0 with 0 velocity
        self.kfy.x=np.array([y, 0])
        self.kfw.x=np.array([w]) # initially at 0,0 with 0 velocity
        self.kfh.x=np.array([h]) # initially at 0,0 with 0 velocity
        #transition matrix (x=x0+v*t)
        self.kfx.F=np.array([ [1.0,1.0],
                [0.0,1.0] ])
        self.kfy.F=np.array([ [1.0,1.0],
                [0.0,1.0] ])
        self.kfw.F=np.array([ [1.0] ])
        self.kfh.F=np.array([ [1.0] ])
        #measurement function (only position)
        self.kfx.H=np.array([[1.0,0.0]])
        self.kfy.H=np.array([[1.0,0.0]])
        self.kfw.H=np.array([[1.0]])
        self.kfh.H=np.array([[1.0]])
        #covariance matrix
        self.kfx.P *= 10. #I guess I figure this is big
        self.kfy.P *= 10. #I guess I figure this is big
        self.kfw.P *= 10. #I guess I figure this is big
        self.kfh.P *= 10. #I guess I figure this is big
        #Fundamental maesurement noise
        self.kfx.R =np.array([[1e-4]])
        self.kfy.R =np.array([[1e-4]])
        self.kfw.R =np.array([[1e-4]])
        self.kfh.R =np.array([[1e-4]])
        self.kfx.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=1e-3)
        self.kfy.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=1e-3)
        self.kfw.Q=np.array( [ [ 1e-3]])
        self.kfh.Q=np.array( [ [ 1e-3]])
        #self.kfw.Q=Q_discrete_white_noise(dim=1,dt=1.0,var=1e-3)
        #self.kfh.Q=Q_discrete_white_noise(dim=1,dt=1.0,var=1e-3)

        self.last_update=[x,y,w,h]
        self.missed_frames=0
        self.seen_frames=0
        self.id=uuid.uuid1()
        self.associated_map_object=None

    def predict(self):
        self.kfx.predict()
        self.kfy.predict()
        self.kfw.predict()
        self.kfh.predict()

    def update(self,x,y,w,h,label):
        self.last_update=[x,y,w,h]

        #print("update tracked object at {} {} {} {} {}".format(x,y,w,h,label))
        self.kfx.update([x])
        self.kfy.update([y])
        self.kfw.update([w])
        self.kfh.update([h])
        self.label=label
        self.missed_frames=0
        self.seen_frames+=1


    def get_predictions(self): #return x,y,w,h
        return self.kfx.x[0],self.kfy.x[0],self.kfw.x[0],self.kfh.x[0]

    def get_extended_bounding_box(self):
        x,y,w,h=self.get_predictions()
        return [x-w/2,x+w/2,y-h/2,y+h/2]

    def get_extended_bounding_box_last_update(self):
        x,y,w,h=self.last_update
        return [x-w/2,x+w/2,y-h/2,y+h/2]

    def get_bounding_box_overlap(self,bbvector):
        myvector=self.get_extended_bounding_box()
        overlap_box=[
            max(bbvector[0],myvector[0]),
            min(bbvector[1],myvector[1]),
            max(bbvector[2],myvector[2]),
            min(bbvector[3],myvector[3])]
        min_area=min( (bbvector[1]-bbvector[0])*(bbvector[3]-bbvector[2]),
(myvector[1]-myvector[0])*(myvector[3]-myvector[2]))
        overlap_area=(overlap_box[1]-overlap_box[0])*(overlap_box[3]-overlap_box[2])
        return overlap_area/min_area

class VisualTracker:
    def __init__(self):
        self.object_tagger=Theo_Chaser_Object_Tagger()
        self.tracked_objects=[]
        #self.id_names_hash=["Franz","Brunhilda","Steve","Persepolis","Legolas","Senator","Doublewide","Convolution","Beaucephalus","Microencephalus","Enchilada","Buttercup","Malferious","Ferrous","Titiana","Europa","Malpractice","Daedelus","Dad-elus","Cheesemonger","Burgertime","ForgetMeNot","Nevar4get","Allowance","Betrayal","Elvis"]
        #self.camera_hfov=48.8*(2*np.pi)/360 #from spec sheet
        self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        self.object_heights={ "stop sign": [0.081,0.005], "sports ball": [0.115,0.01]}

    def get_cost(self,tracked_object,vision_object):
        vision_bbox=[vision_object["startx"],vision_object["endx"],vision_object["starty"],vision_object["endy"]]
        overlap=tracked_object.get_bounding_box_overlap(vision_bbox)
        cost=-overlap
        wrong_class_penalty=0.4
        if vision_object["label"]!=tracked_object.label:
            cost+=wrong_class_penalty
        return cost

    def update(self,frame):
        minimum_track_assignment_threshhold=-0.2
        max_tracked_objects=3
        vision_objects=self.object_tagger.tag_objects(frame)
        #print("{} vision objects {} tracked objects".format(len(vision_objects),len(self.tracked_objects)))
        unused_vision_objects=vision_objects
        for objind in range(len(self.tracked_objects)):
            the_costs=[]
            #predict where the object should be
            self.tracked_objects[objind].predict()
            for visind in range(len(vision_objects)):
                the_costs.append(self.get_cost(self.tracked_objects[objind],unused_vision_objects[visind]))
            best_fit=-1
            if len(the_costs)>=1:
                best_fit=np.argmin(the_costs)
            if best_fit==-1 or the_costs[best_fit]>minimum_track_assignment_threshhold:
                self.tracked_objects[objind].missed_frames+=1
            else:
                #match for object
                xy,wh=self.object_tagger.get_obj_loc_width(unused_vision_objects[best_fit])
                self.tracked_objects[objind].update(xy[0],xy[1],wh[0],wh[1],unused_vision_objects[best_fit]["label"])
                unused_vision_objects.remove(unused_vision_objects[best_fit])
        #any additional vision objects become potential tracks!
        if len(unused_vision_objects)>0 and len(self.tracked_objects)<=max_tracked_objects:
            xy,wh=self.object_tagger.get_obj_loc_width(unused_vision_objects[0])
            new_track=VTTrackedObject(xy[0],xy[1],wh[0],wh[1],unused_vision_objects[0]["label"])
            self.tracked_objects.append(new_track)

        #remove tracks with too many missed frames
        max_missed_frames=10
        to_drop=[]
        for obj in self.tracked_objects:
            if obj.missed_frames>=max_missed_frames:
                to_drop.append(obj)
            elif obj.missed_frames>obj.seen_frames:
                to_drop.append(obj)
        for obj in to_drop:
            self.tracked_objects.remove(obj)

    def draw_bboxes(self,video_frame):
        if video_frame is None:
            logger.warning("Gave me a none video frame")
            return
        for obj in self.tracked_objects:
            sxf,exf,syf,eyf=obj.get_extended_bounding_box()
            sx=320+sxf*640
            sy=240+syf*480
            ex=320+exf*640
            ey=240+eyf*480
            #print("drawing box at {} {} {} {}".format(sx,sy,ex,ey))
            cv.rectangle(video_frame,(int(sx),int(sy)),(int(ex),int(ey)),(0,255,0),2)
            #text = "{} {}".format(obj.label,obj.missed_frames)
            #text = "{} {}".format(self.id_names_hash[hash(obj.id)%len(self.id_names_hash)],obj.missed_frames)
            name="unknown"
            if obj.associated_map_object is not None:
                name=id_to_name(obj.associated_map_object)
            text = "{} {}".format(name,obj.label)
            Y = int(sy - 10 if sy - 10 > 10 else sy + 10)
            cv.putText(video_frame, text, (int(sx),Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
        return video_frame

    def serialize_tracks(self):
        ret=[]
        for obj in self.tracked_objects:
            item={}
            item["label"]=obj.label
            item["id"]=str(obj.id)
            item["last_update"]=obj.last_update
            item["xywh"]=obj.get_predictions()
            ret.append(item)
        return ret

    def retrieve_object(self,id):
        for obj in self.tracked_objects:
            if obj.id==id:
                return obj
        print("{} not found".format(id))
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


    def adjust_all_tracks(self,x,y,w,h,dx,dy,dw,dh):
        #TODO Handle WH better
        for obj in self.tracked_objects:
            obj.kfx.x+=x
            obj.kfx.P[0][0]=obj.kfx.P[0][0]+dx*dx
            obj.kfy.x+=y
            obj.kfy.P[0][0]=obj.kfy.P[0][0]+dy*dy
