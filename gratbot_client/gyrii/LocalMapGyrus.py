from Gyrus import ThreadedGyrus
from underpinnings.BayesianArray import BayesianArray
#from underpinnings.OccupancyMap import OccupancyMap
from underpinnings.OccupancyMap2 import OccupancyMap2,LocalizationException
from underpinnings.OccupancyMap2 import LidarMeasurement, ExactLidarMeasurement
import numpy as np
from uncertainties import ufloat
from uncertainties.umath import *
from uncertainties import unumpy
import scipy
import scipy.signal
import cv2 as cv
import time
from matplotlib import pyplot as plt
from gyrii.underpinnings.GratbotLogger import gprint



class LocalMapGyrus(ThreadedGyrus):
    def __init__(self,broker,resolution=0.05,npoints_x=200,npoints_y=200,debugshow=False,display_loop=None):
        self.resolution=resolution #grid size in meters
        self.npoints_x=npoints_x
        self.npoints_y=npoints_y
        self.debugshow=debugshow
        self.gridmap=OccupancyMap2(resolution,npoints_x,npoints_y)
        self.lidar_localization_score_cutoff=-50
        #self.gridmap=OccupancyMap(resolution,npoints_x,npoints_y)
        #self.gridmap_free=0.9*np.ones([self.npoints_x,self.npoints_y])
        #self.gridmap_occupied=0.1*np.ones([self.npoints_x,self.npoints_y])
        #self.gridmap_prob=0.1*np.ones([self.npoints_x,self.npoints_y])
        self.last_pose=None
        self.last_pose_stable=False
        #self.last_pose=BayesianArray(ndim=3)
        self.state="both" #could be mapping or localization or both.  TODO push this into the messages, and not this state
        self.localization_state="off"
        self.display_loop=display_loop
        self.clear_messages_before=0
        self.mapping_time_spacing=0.05
        self.l_spread_size_sigma=0
        super().__init__(broker)

    def get_keys(self):
        #return [ "latest_pose","ultrasonic_sensor/last_measurement","floor_detector_measurement","lidar/lidar_scan","localization_state" ]
        return [ "latest_pose","lidar/lidar_scan","localization_state" ]

    def get_name(self):
        return "LocalMapGyrus"

    def set_localization_state(self,name):
        self.localization_state=name
        gprint("changing localization state to {}".format(self.localization_state))
        if name=="full":
            self.l_theta_points=48
            self.l_theta_res=np.radians(2.5)
            self.l_spread_size_sigma=0
            self.spread_size_x=40
            self.spread_size_y=40
        elif name=="ultrawide":
            self.l_theta_points=15
            self.l_theta_res=np.radians(2.0)
            #self.l_spread_size_sigma=8
            self.spread_size_x=20
            self.spread_size_y=20
        elif name=="wide":
            self.l_theta_points=5
            self.l_theta_res=np.radians(1.5)
            #self.l_spread_size_sigma=5
            self.spread_size_x=10
            self.spread_size_y=10
        elif name=="close":
            self.l_theta_points=4
            self.l_theta_res=np.radians(1.0)
            #self.l_spread_size_sigma=3
            self.spread_size_x=4
            self.spread_size_y=4
        elif name=="off":
            pass
        else:
            gprint("ERROR BAD LOCALIZATION STATE: {}".format(name))
    def read_message(self,message):
        ret=[]
        if "latest_pose" in message:
            self.last_pose=BayesianArray.from_object(message["latest_pose"])
            if message["pose_notes"]=="pose_is_stable":
                self.last_pose_stable=True
            else:
                self.last_pose_stable=False
        if "localization_state" in message:
            self.set_localization_state(message["localization_state"])
        if "lidar/lidar_scan" in message:
            if self.last_pose==None:
                return [] #useless until I've gotten a pose update
            if message["timestamp"]<self.clear_messages_before:
                return #drop scans if I get too far behind
            #gprint("scanning")
            scan_data=message["lidar/lidar_scan"]
            m=ExactLidarMeasurement([ x[2]/1000 for x in scan_data ],[ np.radians(x[1]) for x in scan_data ])
            #angles=[ np.radians(x[1]) for x in scan_data ]
            #angle_unc=np.radians(1.0)*np.ones(len(angles))
            #dists=[ x[2]/1000 for x in scan_data ]
            #dist_uncs=0.01*np.ones(len(angles))
            #m=LidarMeasurement(dists,dist_uncs,angles,angle_unc)
            innovation_sigma=0 #for keeping track of how far I disagree from current pos
            localization_exception=False

            if self.localization_state!="off":
                start_time=time.time()
                posemap,xs,ys,ts,counts=self.gridmap.get_lidar_pose_map3_withtheta(self.last_pose.vals,m,theta_points=self.l_theta_points,theta_res=self.l_theta_res)
                try:
                    if self.l_spread_size_sigma>0:
                        #gprint("{} {}".format(self.last_pose.covariance[0][0],self.last_pose.covariance[1][1]))
                        sigma=np.sqrt(max(self.last_pose.covariance[0][0],self.last_pose.covariance[1][1]))
                        self.spread_size_x=max(int(sigma*self.l_spread_size_sigma/self.gridmap.resolution),2)
                        self.spread_size_y=self.spread_size_x
                    newpose,center_score=self.gridmap.pose_map_to_pose_prediction3_with_theta_faster(posemap,xs,ys,ts,counts,center=self.last_pose,spread_size_x=self.spread_size_x,spread_size_y=self.spread_size_y)
                    last_pose_copy=self.last_pose.copy()
                    last_pose_copy.covariance+=np.diag([0.05**2,0.05**2,0.01**2])
                    innovation_sigma=np.sqrt(last_pose_copy.chi_square_from_point(newpose.vals))
                    gprint("localization type: {} spreadsize {} score: {} innovation_sigma {}".format(self.localization_state,self.spread_size_x,center_score,innovation_sigma))
                    if np.isnan(innovation_sigma):
                        raise LocalizationException("sigma is NaN")
                    if innovation_sigma>10 and self.localization_state!="full":
                        raise LocalizationException("sigma too high")
                    if innovation_sigma<5 and self.localization_state=="ultrawide":
                        self.set_localization_state("wide")
                    elif innovation_sigma<5 and self.localization_state=="wide":
                        self.set_localization_state("close")
                    dt=time.time()-start_time
                    #wrap angle
                    gprint("Localizing to {}".format(newpose.pretty_str()))
                    self.broker.publish({"pose_measurement": newpose.to_object(),"timestamp": message["timestamp"],"notes": "lidar"},"pose_measurement")
                except LocalizationException as e:
                        gprint("Localization Exception: {}".format(e.message))
                        gprint("Pose is : {}".format(self.last_pose.pretty_str()))
                        localization_exception=True
                        if self.localization_state=="close":
                            self.set_localization_state("wide")
                        elif self.localization_state=="wide":
                            self.set_localization_state("ultrawide")
                        else:
                            gprint("Uh oh, already at {}".format(self.localization_state))

            if (self.state=="both" or self.state=="mapping") and self.last_pose_stable and (not localization_exception):
                self.gridmap.apply_exact_lidar_measurement3(self.last_pose.vals,m,weight=1)
                #self.gridmap.apply_lidar_measurement(self.last_pose,m) #record map
            self.clear_messages_before=time.time()+self.mapping_time_spacing


        if "ultrasonic_sensor/last_measurement" in message:
            return
            if self.last_pose==None:
                return [] #useless until I've gotten a pose update
            #if message["ultrasonic_sensor/last_measurement"]["average_distance"]>2.0:
            if message["ultrasonic_sensor/last_measurement"]["average_distance"]>1.5:
                return [] #ultrasonic measure becomes untrustworthy beyond a meter and a bit
            #gprint("Ultrasonic Distance: {}".format(message["ultrasonic_sensor/last_measurement"]["average_distance"]))
            return []
            angle_unc=7*2*np.pi/360 #+/- 7 degree uncertainty for ultrasonic measurement
            #this works, btu I can't figure out what to use it for
            #range_guess=self.gridmap.guess_range_to_wall(self.last_pose)
            #lsc=self.gridmap.delta_logsum_explored_in_cone(self.last_pose,range_guess,-angle_unc,+angle_unc)
            #print("range guess {}, measured {}".format(range_guess,message["ultrasonic_sensor/last_measurement"]["average_distance"]))
            #print("LogSumExplored {}".format(lsc))
            m=LidarMeasurement([message["ultrasonic_sensor/last_measurement"]["average_distance"]],[message["ultrasonic_sensor/last_measurement"]["stdev_distance"]],[0],[angle_unc])
            if self.state=="both" or self.state=="mapping":
                self.gridmap.apply_lidar_measurement(self.last_pose,m) #record map
            if False: #if self.state=="both" or self.state=="localization":
                xs,ys,mymap=self.gridmap.calculate_eval_map(self.last_pose.vals,[0.5,0.5,0],m)
                if self.debugshow:
                    print("xs and ys {} {}".format(xs,ys))
                ret.append({"timestamp": message["timestamp"],"offsetmap": [mymap,xs,ys]})
                pose_no_cov=self.gridmap.eval_map_to_point(xs,ys,mymap)
                try:
                    #print("Ultra Pose estimate {},{}".format(pose_no_cov.get_as_ufloat()[0],pose_no_cov.get_as_ufloat()[1]))
                    self.broker.publish({"pose_measurement": pose_no_cov.to_object(),"timestamp": message["timestamp"],"notes": "ultrasonic_map"},"pose_measurement")
                except:
                    print("error converting ultrapose estimate {}".format(pose_no_cov))
        if "floor_detector_measurement" in message:
            return
            if self.last_pose==None:
                return [] #useless until I've gotten a pose update
            mes=message["floor_detector_measurement"]
            #this part marks up my map
            m=LidarMeasurement(mes["dists"],mes["dists_unc"],mes["x_angles"],mes["x_angles_unc"])
            #gprint("Lidar Distance: {}".format(mes["dists"][int(len(mes["dists"])/2)]))
            last_pose=BayesianArray.from_object(mes["last_pose"]) #NOTE THIS IS DIFFERENT FROM WHAT THE CURRENT POSE IS
            #gprint("Local Map Pose {}".format(last_pose.get_as_ufloat()))
            if self.state=="both" or self.state=="localization":
                xs,ys,posemap=self.gridmap.get_lidar_pose_map(last_pose.vals,m.to_exact(),5,5)
                if np.max(posemap)>self.lidar_localization_score_cutoff:
                    #gprint("min max posemap {} {}".format(np.min(posemap),np.max(posemap)))
                    pred=self.gridmap.pose_map_to_pose_prediction(xs,ys,posemap)
                    min_unc=self.gridmap.resolution
                    min_pred_cov=np.array([[min_unc*min_unc,0,0],
                                           [0,min_unc*min_unc,0],
                                           [0,0,0]])
                    pred.covariance+=min_pred_cov
                    self.broker.publish({"pose_measurement": pred.to_object(),"timestamp": message["timestamp"],"notes": "floor lidar"},"pose_measurement")
            if self.state=="both" or self.state=="mapping":
                self.gridmap.apply_lidar_measurement(last_pose,m) #record map

    def get_pose_measurement_logprob(self,pose,measurement):
        points=measurement.to_xypoints(pose)
        ret=0
        for p in points:
            coord=self.coord_to_cell(p.vals)
            #print("point {} yields {}".format(coord,self.gridmap_occupied[coord[0],coord[1]]))
            ret+=self.gridmap_occupied[coord[0],coord[1]]
        return ret

#    def make_evaluation_map(self,measurement):
#        test_pose=self.last_pose.copy()
#        test_pose.covariance=np.zeros([3,3])
#        points=measurement.to_xypoints(test_pose)
#        #TODO expand to handle multiple points
#        center_cell,area=self.cov_pt_toregion(points[0],ncells=2)
#        submap_size=32
#        #sub_map=self.gridmap_occupied[center_cell[0]-submap_size:center_cell[0]+submap_size,center_cell[1]-submap_size:center_cell[1]+submap_size]
#        sub_map=self.gridmap_prob[center_cell[0]-submap_size:center_cell[0]+submap_size,center_cell[1]-submap_size:center_cell[1]+submap_size]
#        #,max_ncells=2)
#        ret= scipy.signal.convolve2d(area,sub_map,mode='valid')
#        return ret
#
    #def emap_to_point(self,emap):
    #    sum_x=0
    #    sum_y=0
    #    sum_xx=0
    #    sum_xy=0
    #    sum_yy=0
    #    sum_p=0
    #    d=(emap.shape[0]-1)/2
    #    offset=np.min(emap)
    #    #print(emap)
    #    for i in range(emap.shape[0]):
    #        for j in range(emap.shape[1]):
    #            x=(i-d)*self.resolution
    #            y=(j-d)*self.resolution
    #            #print(x)
    #            p=emap[i,j]-offset
    #            sum_p+=p
    #            sum_x+=p*x
    #            sum_y+=p*y
    #            sum_yy+=p*y*y
    #            sum_xx+=p*x*x
    #            sum_xy+=p*x*y
    #    xmean=sum_x/sum_p
    #    ymean=sum_y/sum_p
    #    xxmean=sum_xx/sum_p-xmean*xmean
    #    yymean=sum_yy/sum_p-ymean*ymean
    #    xymean=sum_xy/sum_p-xmean*ymean
    #    return BayesianArray(np.array([xmean,ymean,0]),np.array([[xxmean,xymean,0],[xymean,yymean,0],[0,0,1000]]))
    #    #print("best guess point {},{}".format(ufloat(xmean,sqrt(xxmean)),ufloat(ymean,sqrt(yymean))))
