#LocalMapGyrus2
from gyrii.underpinnings.GratbotLogger import gprint

#from underpinnings.OccupancyMap2 import LidarMeasurement, ExactLidarMeasurement
from gyrii.underpinnings.OccupancyMap2 import OccupancyMap2,LocalizationException,LidarMeasurement, ExactLidarMeasurement
from underpinnings.BayesianArray import BayesianArray
from Gyrus import ThreadedGyrus
import numpy as np
import time

class LocalMapGyrus(ThreadedGyrus):
    def __init__(self,broker,shared_objects,display_loop):

        self.localization_state="off"
        self.clear_messages_before=0
        self.clear_time_spacing=0.025
        self.last_map_time=0
        self.mapping_time_spacing=0.2


        #self.last_pose=None
        self.last_pose=BayesianArray(np.array([0,0,0]),100.0*np.eye(3))
        self.last_pose_stable=False

        self.loud=False
        #self.loud=True

        #for the navigation grid map
        self.update_navigation_every_n_seconds=10
        self.last_navigation_map_update=0

        #for localization
        self.spread_size=0 #how many positions to look over.  multiply by resolution to see how far I'm checking
        self.local_lidar_map_size=64 #how big to use for a localization.  For a resolution of 0.1 m this is 6.4 m wide
        super().__init__(broker,shared_objects)

    def load_config(self,config):
        try:
            myconfig=config["LocalMap"]
            mapname=myconfig["map_filename"]
            with self.shared_objects.locks["occupancy_map"]:
                self.shared_objects.objects["occupancy_map"]=OccupancyMap2.load_from_file(mapname,res=float(myconfig["resolution"]))
            self.set_localization_state("full")
            gprint("loaded occupancy map")

        except Exception as e:
            gprint("Failed to load local map config ({}), starting from scratch".format(e))


    def get_keys(self):
        return [ "latest_pose","lidar/lidar_scan","localization_state" ]

    def get_name(self):
        return "LocalMapGyrus"

    def set_localization_state(self,name):
        self.localization_state=name
        gprint("changing localization state to {}".format(self.localization_state))
        if name=="full":
            self.l_theta_points=36
            self.l_theta_res=np.radians(5.0)
            #self.l_spread_size_sigma=0
            self.spread_size=40
        elif name=="ultrawide":
            self.l_theta_points=15
            self.l_theta_res=np.radians(6.0)
            #self.l_spread_size_sigma=o
            self.spread_size=15
            #self.spread_size_x=20
            #self.spread_size_y=20
        elif name=="wide":
            self.l_theta_points=5
            self.l_theta_res=np.radians(4.0)
            #self.l_spread_size_sigma=5
            self.spread_size=10
            #self.spread_size_x=20
            #self.spread_size_y=20
        elif name=="close":
            self.l_theta_points=5
            self.l_theta_res=np.radians(2.5)
            #self.l_spread_size_sigma=3
            #self.spread_size_x=4
            #self.spread_size_y=4
            self.spread_size=10
        elif name=="off":
            pass
        else:
            gprint("ERROR BAD LOCALIZATION STATE: {}".format(name))
        self.global_map_halfsize=int(self.local_lidar_map_size/2+self.spread_size)

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
            mdown=m.downsample(2)
            #angles=[ np.radians(x[1]) for x in scan_data ]
            #angle_unc=np.radians(1.0)*np.ones(len(angles))
            #dists=[ x[2]/1000 for x in scan_data ]
            #dist_uncs=0.01*np.ones(len(angles))
            #m=LidarMeasurement(dists,dist_uncs,angles,angle_unc)
            innovation_sigma=0 #for keeping track of how far I disagree from current pos
            localization_exception=False

            if self.localization_state!="off":
                start_time=time.time()
                with self.shared_objects.locks["occupancy_map"]:
                    gridmap=self.shared_objects.objects["occupancy_map"]

                    posemap,xs,ys,ts,counts=gridmap.get_lidar_pose_map5_withtheta(self.last_pose.vals,mdown,theta_points=self.l_theta_points,theta_res=self.l_theta_res,local_lidar_map_size=self.local_lidar_map_size,global_map_halfsize=self.global_map_halfsize)
                    innovation_sigma=5
                    try:
                        #if self.l_spread_size_sigma>0:
                            #gprint("{} {}".format(self.last_pose.covariance[0][0],self.last_pose.covariance[1][1]))
                        #    sigma=np.sqrt(max(self.last_pose.covariance[0][0],self.last_pose.covariance[1][1]))
                        #    self.spread_size_x=max(int(sigma*self.l_spread_size_sigma/gridmap.resolution),2)
                        #    self.spread_size_y=self.spread_size_x
                        newpose,center_score=gridmap.pose_map_to_pose_prediction4_with_theta_faster(posemap,xs,ys,ts,counts,center=self.last_pose,spread_size_x=self.spread_size,spread_size_y=self.spread_size)
                        last_pose_copy=self.last_pose.copy()
                        innovation=self.last_pose-newpose
                        innovation_sigma=self.last_pose.chi_square_from_pose(newpose)
                        #last_pose_copy.covariance+=np.diag([0.05**2,0.05**2,0.01**2])
                        #innovation_sigma=np.sqrt(last_pose_copy.chi_square_from_point(newpose.vals))
                        if self.loud or self.localization_state=="full":
                            gprint("localization type: {} spreadsize {} score: {} innovation_sigma {}".format(self.localization_state,self.spread_size,center_score,innovation_sigma))
                            gprint("From Pose : {}".format(self.last_pose.pretty_str()))
                            gprint("Localizing to {}".format(newpose.pretty_str()))
                        if np.isnan(innovation_sigma):
                            raise LocalizationException("sigma is NaN")
                        #if innovation_sigma>10 and self.localization_state!="full":
                        #    raise LocalizationException("sigma too high: {} -> {}".format(newpose.pretty_str(),innovation_sigma))
                        if innovation_sigma<5 and self.localization_state=="full" and innovation.vals[2]<np.pi/8 and innovation.vals[0]<0.2 and innovation.vals[1]<0.2:
                            self.set_localization_state("wide")
                        #elif innovation_sigma<5 and self.localization_state=="ultrawide":
                        elif self.localization_state=="ultrawide":
                            self.set_localization_state("wide")
                        #elif innovation_sigma<5 and self.localization_state=="wide":
                        elif self.localization_state=="wide":
                            self.set_localization_state("close")
                        dt=time.time()-start_time
                        #wrap angle
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
                                gprint("Uh oh, already at {}, going back to close".format(self.localization_state))
                                self.set_localization_state("close")

            #if (self.state=="both" or self.state=="mapping") and self.last_pose_stable and (not localization_exception) and (innovation_sigma<3):
            if (self.state=="both" or self.state=="mapping") and (not localization_exception) and (innovation_sigma<3) and message["timestamp"]>self.last_map_time+self.mapping_time_spacing and self.last_pose.covariance[0][0]<0.01 and self.last_pose.covariance[1][1]<0.01:
                with self.shared_objects.locks["occupancy_map"]:
                    gridmap=self.shared_objects.objects["occupancy_map"]
                    gridmap.apply_exact_lidar_measurement3(self.last_pose.vals,m,weight=1)
                    #update the navigational map every so often
                self.last_map_time=message["timestamp"]
            #if time.time()>self.last_navigation_map_update+self.update_navigation_every_n_seconds:
            #    gprint("updating graph map")
            #    with self.shared_objects.locks["occupancy_map"]:
            #        with self.shared_objects.locks["graph_map"]:
            #            gridmap=self.shared_objects.objects["occupancy_map"]
            #            graphmap=self.shared_objects.objects["graph_map"]
            #            graphmap.describe_occupancy_map(gridmap)
            #    self.last_navigation_map_update=time.time()


            self.clear_messages_before=time.time()+self.clear_time_spacing
