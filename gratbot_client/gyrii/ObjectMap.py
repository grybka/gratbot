
from Gyrus import ThreadedGyrus
from uncertainties import ufloat,umath
import numpy as np
import cv2 as cv
from underpinnings.id_to_name import id_to_name
from underpinnings.BayesianArray import BayesianArray
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import uuid
from gyrii.underpinnings.GratbotLogger import gprint, gprint_low
from scipy.optimize import linear_sum_assignment

class StaticObjectTrack:
    def __init__(self,position,visualid,objectlabel,timestamp):
        self.last_update_timestamp=timestamp
        self.visual_id=visualid
        self.object_label=objectlabel
        self.id=uuid.uuid1()
        self.position=position.copy() #should be a BayesianArray
        self.min_unc=0.01
        #self.noise_per_dt=0.03

#    def __init__(self,position,visualid,objectlabel,timestamp):
        #self.position=position #should be a BayesianArray
#        self.positionfilter = KalmanFilter(dim_x=2,dim_z=2) #x,y
#        self.positionfilter.x =position.vals.T
#        self.positionfilter.P = position.covariance
#        self.positionfilter.F=np.array([ [1.0,0.0], [0.0,1.0] ] )
#        self.positionfilter.H=np.array([[1.0,0.0],[0.0,1.0]])
#        self.positionfilter.R =np.array([[1,0],
#                              [0,1]])
#        self.positionfilter.Q=Q_discrete_white_noise(dim=2,dt=1.0,var=1e-2)
#        self.last_update_timestamp=timestamp
#        self.visual_id=visualid
#        self.object_label=objectlabel
#        self.id=uuid.uuid1()

    def update_position_with_measurement(self,position):
#        self.positionfilter.R = position.covariance
#        self.positionfilter.update(position.vals.T)
        self.position=self.position.updated(position)

    def predict_to(self,timestamp):
        if self.last_update_timestamp>timestamp:
            return
        dt=timestamp-self.last_update_timestamp
        if self.position.covariance[0][0]<self.min_unc**2:
            self.position.covariance[0][0]=self.min_unc**2
        if self.position.covariance[1][1]<self.min_unc**2:
            self.position.covariance[1][1]=self.min_unc**2
#        self.positionfilter.Q=Q_discrete_white_noise(dim=2,dt=dt,var=1e-2)
#        self.positionfilter.predict()
        self.last_update_timestamp=timestamp

    def getpos(self):
        #gprint("self.positionfilter.x {}".format(self.positionfilter.x))
        #return BayesianArray(np.array([self.positionfilter.x[0],self.positionfilter.x[1]]),self.positionfilter.P)
        return self.position

    def get_measurement_nsigma_away(self,other_measurement):
        pos1=self.position.get_as_ufloat()
        pos2=other_measurement.get_as_ufloat()
        delta=pos1-pos2
        dist=umath.sqrt(delta[0]*delta[0]+delta[1]*delta[1])
        nsigma=dist.n/dist.s
        return nsigma



class ObjectMapGyrus(ThreadedGyrus):
    def __init__(self,broker,debugshow=False,display_loop=None):
        self.display_loop=display_loop
        self.static_objects=[]
        self.object_heights={ "stop sign": [0.081,0.005], "sports ball": [0.115,0.01], "chair": [1.0,0.5]}
        self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        super().__init__(broker)


    def get_keys(self):
        #return [ "latest_pose","visual_tracker_objects" ]
        return [ "visual_tracker_objects" ]

    def get_name(self):
        return "ObjectMapGyrus"

    def visual_xywh_to_map_xy(self,xywh,dxdydwdh,label,pose):
        #TODO move these somewhere nice
        self.video_width=640
        self.video_height=480
        self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        pose_angle=ufloat(pose.vals[2],np.sqrt(pose.covariance[2][2]))
        angle_from_view=ufloat(   self.camera_wfov*(xywh[0]-self.video_width/2)/self.video_width, self.camera_wfov*dxdydwdh[2]/self.video_width )
        angle=pose_angle+angle_from_view
        dist_guess=ufloat(2,3)
        if label in self.object_heights:
            #h=self.object_heights[label][0]
            #dh=self.object_heights[label][1]
            h=ufloat(self.object_heights[label][0],self.object_heights[label][1])
            pred=ufloat(xywh[3],dxdydwdh[3])
            dist_guess=h/umath.tan(pred*self.camera_hfov/self.video_height)
        mapx=dist_guess*umath.sin(angle)
        mapy=dist_guess*umath.cos(angle)
        return BayesianArray.from_ufloats(np.array([mapx,mapy]))

    def get_objects_in_view_cone(self):
        ret=[]
        #return array of static objects
        for sobj in self.static_objects:
            angle_to_obj=np.arctan2(sobj.position.vals[0]-self.last_pose.vals[0],sobj.position.vals[1]-self.last_pose.vals[1])
            delta_angle=angle_to_obj-self.last_pose.vals[2]
            delta_angle = ( delta_angle + np.pi) % (2 * np.pi ) - np.pi
            #gprint("delta angle {}".format(np.degrees(delta_angle)))
            if abs(delta_angle)<self.camera_wfov/2:
                ret.append(sobj)
        return ret

    def get_cost(self,visobj,myobj):
        wrong_label_cost=3.0
        obj_pos=self.visual_xywh_to_map_xy(visobj["xywh"],visobj["dxdydwdh"],visobj["label"],self.last_pose)
        nsigma_away=myobj.get_measurement_nsigma_away(obj_pos)
        cost=0
        if visobj["label"]!=myobj.object_label:
            cost+=wrong_label_cost
        return cost+nsigma_away

    def assign_objects(self,visual_objects):
        objects_to_see=self.get_objects_in_view_cone()
        assigned_pairs=[]
        orphaned_objects_to_see=[]
        orphaned_visual_objects=[]
        cost_matrix=np.zeros([len(objects_to_see),len(visual_objects)])
        for i in range(len(objects_to_see)):
            for j in range(len(visual_objects)):
                cost_matrix[i,j]=self.get_cost(visual_objects[j],objects_to_see[i])
        row_ind,col_ind=linear_sum_assignment(cost_matrix)
        for i in range(len(objects_to_see)):
            if i not in row_ind:
                orphaned_objects_to_see.append(objects_to_see[i])
        for i in range(len(visual_objects)):
            if i not in col_ind:
                orphaned_visual_objects.append(visual_objects[i])
        for i in range(len(row_ind)):
            sobj=objects_to_see[row_ind[i]]
            obj=visual_objects[col_ind[i]]
            assigned_pairs.append([sobj,obj])
        return assigned_pairs,orphaned_objects_to_see,orphaned_visual_objects

    def read_message(self,message):
        ret=[]
        #if "latest_pose" in message:
        #    self.last_pose=BayesianArray.from_object(message["latest_pose"])
        if "visual_tracker_dropped_objects" in message:
            for id in message["visual_tracker_dropped_objects"]:
                for sobj in self.static_objects:
                    if sobj.visual_id==id:
                        sobj.visual_id=None
        if "visual_tracker_objects" in message:
            #predict what I -should- see
            self.last_pose=BayesianArray.from_object(message["pose_used"])
            timestamp=message["timestamp"]
            found_objects=[]
            visual_objects=message["visual_tracker_objects"]
            assigned_pairs,orphaned_objects_to_see,orphaned_visual_objects=self.assign_objects(visual_objects)
            gprint("Assigned pairs {}, orphaned objects to see {},orphaned visual objects {}".format(len(assigned_pairs),len(orphaned_objects_to_see),len(orphaned_visual_objects)))
            for sobj,obj in assigned_pairs:
                obj_pos=self.visual_xywh_to_map_xy(obj["xywh"],obj["dxdydwdh"],obj["label"],self.last_pose)
                sobj.predict_to(timestamp)
                sobj.update_position_with_measurement(obj_pos)
            for sobj in orphaned_objects_to_see:
                gprint("could not find {} {}, where did it go?".format(sobj.object_label,id_to_name(sobj.id)))
            for obj in orphaned_visual_objects:
                obj_pos=self.visual_xywh_to_map_xy(obj["xywh"],obj["dxdydwdh"],obj["label"],self.last_pose)
                gprint("new object {} at {},{}".format(obj["label"],obj_pos.vals[0],obj_pos.vals[1]))
                new_obj=StaticObjectTrack(obj_pos,obj["id"],obj["label"],timestamp)
                self.static_objects.append(new_obj)
            if self.display_loop is not None:
                self.display_object_map()

    def display_object_map(self):
        xsize=640
        ysize=480
        scale=xsize/4
        toshow=np.zeros([ysize,xsize])
        for sobj in self.static_objects:
            position=sobj.getpos()
            x,y,h1,h2,theta=position.get_error_ellipse()
            x=x-self.last_pose.vals[0]
            y=y-self.last_pose.vals[1]
            ex=int(x*scale+xsize/2)
            ey=int(-y*scale+ysize/2)
            #gprint("it's at {} {}".format(ex,ey))
            color = (150, 150, 150)
            thickness = 2
            theta=-np.degrees(theta)
            cv.ellipse(toshow,(ex,ey),( int(h1*scale),int(h2*scale)),theta,0,360,color,thickness)
            samps=position.random_sample(size=100)
            #for s in samps:
            #    exx=int(s[0]*scale+xsize/2)
            #    eyy=int(-s[1]*scale+ysize/2)
            #    center_coordinates = (exx, eyy)
            #    radius = 2
            #    color = (255, 0, 0)
            #    thickness = 1
            #    toshow = cv.circle(toshow, center_coordinates, radius, color, thickness)

            fontScale = 0.4
            color = (255, 255, 255)
            # Line thickness of 2 px
            thickness = 1
            font = cv.FONT_HERSHEY_SIMPLEX
            text="{} ({})".format(sobj.object_label,id_to_name(sobj.id))
            org=( ex,ey)
            toshow = cv.putText(toshow, text, org, font,  fontScale, color, thickness, cv.LINE_AA)

        startpt_x=int(self.last_pose.vals[0]*scale+xsize/2)
        startpt_y=int(-self.last_pose.vals[1]*scale+ysize/2)
        arrowpt_x=int(self.last_pose.vals[0]*scale+xsize/2+10*np.sin(self.last_pose.vals[2]))
        arrowpt_y=int(-self.last_pose.vals[1]*scale+ysize/2-10*np.cos(self.last_pose.vals[2]))
        #gprint("self.last_pose {}".format(self.last_pose.vals[2]))

        #self.coord_to_cell(self.last_pose.vals[0:2]+np.array([0.2*sin(pose.vals[2]),0.2*cos(self.last_pose.vals[2])]))
        #startpt=(xsize/2,ysize/2) #remember I have to flip the y axis
        #arrowpt=self.coord_to_cell(self.last_pose.vals[0:2]+np.array([0.2*sin(pose.vals[2]),0.2*cos(self.last_pose.vals[2])]))
        cv.arrowedLine(toshow,(startpt_x,startpt_y),(arrowpt_x,arrowpt_y),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        self.display_loop.update_image("Objectmap",toshow)
