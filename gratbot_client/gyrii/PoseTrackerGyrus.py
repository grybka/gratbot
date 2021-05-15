
from Gyrus import ThreadedGyrus
from underpinnings.BayesianArray import BayesianArray
import time
import numpy as np
from gyrii.underpinnings.KalmanPoseFilter import KalmanPoseFilter
from gyrii.underpinnings.GratbotLogger import gprint

class PoseTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.pose=KalmanPoseFilter(1e-3,1,1e-3,1e-3)
        self.pose.q_variance=10
        self.pose.last_time_update=time.time()
        #TODO what is my q variance noise on this?
        self.last_message_sent_time=0
        self.spam_message_time=0.05
        self.reject_updates_before=time.time()
        self.last_unstable_pose_time=0
        self.pose_stabilitiy_wait_time=0.5 #wait this long before declaring a pose stable

        #self.pose=BayesianArray(np.zeros(3),np.array([[1e-6,0,0],[0,1e-6,0],[0,0,100]]))
        #self.velocity=BayesianArray(ndim=3)
        #self.last_timestamp=time.time()
        #self.min_time_update=1e-2
        #regular kalman filters use a process variance that adds with time
        #I'm instead going to stick with a minimum variance
        #self.pose_min_covariance=np.array([0.05**2,0.05*0.05,0.05**2]) #5cm, 3 degrees
        #self.pose_min_covariance=np.array([0.02**2,0.02*0.05,0.01**2]) #2cm, 1 degrees
        #self.pose_how_often=0.1
        #self.time_pose_record=[ [time.time(),self.pose] ]
        super().__init__(broker)
        #self.velocity_min_covariance=np.array([5e-3**2,5e-3**2,9e-4**2]) #0.5 cm/s , 0.5 degree per second
        #the other thing this doesn't do that a full kalman filter would is
        #infer the velocity from a changing position.  I'm not sure I actually want it to do that
        #because then map updates would make it think it was moving

    #def update_to_time(self,timestamp):
    #    if timestamp<self.last_timestamp+self.min_time_update:
    #        return #don't update if no real time has passed
    #    delta_t=timestamp-self.last_timestamp
    #    #x=x1+v*t
    #    self.pose.vals+=self.velocity.vals*delta_t
    #    self.pose.covariance+=self.velocity.covariance*delta_t
    #    #covariance can only increase so don't bother checking process noise
    #    self.last_timestamp=timestamp



    def load_config(self,config):
        print("load config called")
        gprint("load config called")
        try:
            myconfig=config["Pose"]
            location=myconfig["location"]
            uncertainty=myconfig["uncertainty"]
            self.pose.pose.vals[0]=location[0]
            self.pose.pose.vals[2]=location[1]
            self.pose.pose.vals[4]=location[2]
            self.pose.pose.covariance[0,0]=uncertainty[0]**2
            self.pose.pose.covariance[2,2]=uncertainty[1]**2
            self.pose.pose.covariance[4,4]=uncertainty[2]**2
            gprint("loaded starting location {}+-{}".format(location, uncertainty))
        except Exception as e:
            self.pose.pose.vals[0]=0
            self.pose.pose.vals[2]=0
            self.pose.pose.vals[4]=0
            self.pose.pose.covariance[0,0]=1e-6
            self.pose.pose.covariance[2,2]=1e-6
            self.pose.pose.covariance[4,4]=1e-6
            gprint("Failed to pose config ({}), starting from scratch".format(e))

    def get_keys(self):
        return [ "pose_measurement","velocity_measurement","pose_certainty_lost","pose_is_changing" ]

    def get_name(self):
        return "PoseTrackerGyrus"

    def get_pose_message(self,timestamp=None):
        if timestamp==None:
            timestamp=time.time()
        if timestamp-self.last_message_sent_time<self.spam_message_time:
            return

#       twodegrees=np.radians(2)
        m={"timestamp": timestamp,"latest_pose": self.pose.get_xytheta().to_object(),"latest_pose_vel": self.pose.get_vxvyvtheta().to_object(),"pose_notes": "pose_unstable"}
        #TODO reintroduce stability condition here
        twodegrees=np.radians(2)
        vx=self.pose.pose.vals[1]
        vy=self.pose.pose.vals[3]
        vtheta=self.pose.pose.vals[5]
        dx=np.sqrt(self.pose.pose.covariance[0][0])
        dy=np.sqrt(self.pose.pose.covariance[2][2])
        dtheta=np.sqrt(self.pose.pose.covariance[4][4])
        if dtheta<twodegrees and abs(vx)<0.05 and abs(vy)<0.05 and abs(vtheta)<0.05 and dx<0.09 and dy<0.09:
            if timestamp>self.last_unstable_pose_time+self.pose_stabilitiy_wait_time:
                m["pose_notes"]="pose_is_stable"
            else:
                m["pose_notes"]="pose_unstable for another {} s".format(self.last_unstable_pose_time+self.pose_stabilitiy_wait_time-timestamp)
        else:
            self.last_unstable_pose_time=timestamp
        self.broker.publish(m,"latest_pose")
        self.last_message_sent_time=timestamp
        #gprint("publishing pose {}".format(self.pose.get_xytheta().pretty_str()))




        #conditions for a pose to be considered stable
        #1)  hasn't changed in the last half second
        #2)  uncertainty is no more than 10 percent greater than min uncertainty
#        self.time_pose_record.append([ timestamp,self.pose])
#        while (timestamp-self.time_pose_record[0][0])>2: #remove anything beyond a full two second
#            self.time_pose_record.pop(0)
#        xs=[ x[1].vals[0] for x in self.time_pose_record]
#        ys=[ x[1].vals[1] for x in self.time_pose_record]
#        ts=[ x[1].vals[2] for x in self.time_pose_record]
#        dx=np.max(xs)-np.min(xs)
#        dy=np.max(ys)-np.min(ys)
#        dt=np.max(ts)-np.min(ts)
#        twodegrees=np.radians(2)
#        if abs(dx)<0.02 and abs(dy)<0.02 and abs(dt)<twodegrees and np.sqrt(self.pose.covariance[2][2])<twodegrees:
#            m={"timestamp": timestamp,"latest_pose": self.pose.to_object(),"pose_notes": "pose_is_stable"}
#            self.broker.publish(m,"latest_pose")
#        else:
#            #gprint("pose not stable {} {} {} {}".format(abs(dx),abs(dy),abs(dt), np.sqrt(self.pose.covariance[2][2]) ))
            #gprint("vs not stable {} {} {} {}".format(0.02,0.02,twodegrees,twodegrees))
            #gprint("pose t is {}".format(self.pose.vals[2]))
#            m= {"timestamp": timestamp,"latest_pose": self.pose.to_object(),"pose_notes": "none"}
#            self.broker.publish(m,"latest_pose")

    def read_message(self,message):
        #TODO suppose I receive a pose measurement with a timestamp from before the last pose offset
        #should I be able to figure out how to update that measurement?
        #if "velocity_measurement" not in message:
        #    gprint("pose meassage received {}".format(message.keys()))
        if "pose_is_changing" in message:
            self.reject_updates_before=message["timestamp"]
        if "pose_measurement" in message:
            if message["timestamp"]<self.reject_updates_before:
                gprint("rejecting pose, too late")
                return
            update=BayesianArray.from_object(message["pose_measurement"])
            #wrap carefully here so as to not produce bizzare swings
            #gprint("updating pose with {}".format(update.pretty_str()))
            #gprint("before pose update {}".format(self.pose.getxytheta().pretty_str()))

            #gprint("pose update {}".format(update.pretty_str()))
            #gprint("vals {}".format(update.vals))
            #gprint("covariance {}".format(update.covariance))
            #gprint("preupdate {}".format(self.pose.get_xytheta().pretty_str()))
            #gprint("preupdate vals {}".format(list(self.pose.pose.vals)))
            #gprint("preupdate covariance {}".format(list(self.pose.pose.covariance)))
            self.pose.update_with_pose_measurement(message["timestamp"],update)
            #gprint("postupdate {}".format(self.pose.get_xytheta().pretty_str()))
            self.get_pose_message(timestamp=message["timestamp"])
        if "velocity_measurement" in message:
            if message["timestamp"]<self.reject_updates_before:
                #gprint("rejecting velocity, too late")
                return
            update=BayesianArray.from_object(message["velocity_measurement"])
#            gprint("velocity mesurement update {}".format(update.pretty_str()))
            #gprint("updating velocity with {}".format(update.pretty_str()))
            self.pose.update_with_velocity_measurement(message["timestamp"],update)
            self.get_pose_message(timestamp=message["timestamp"])
        if "pose_certainty_lost" in message:
            gprint("Pose certainty lost!")
            #I guess I have these hardcoded.  Lost means 50 cm uncertainty in position, pi uncertainty in angle
            #maybe will need to refine it when it is used for transition between local maps
            self.reject_updates_before=message["timestamp"]
            self.pose.declare_position_uncertainty(0.50,np.pi)
