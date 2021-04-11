
from Gyrus import ThreadedGyrus
from underpinnings.BayesianArray import BayesianArray
import time
import numpy as np
from gyrii.underpinnings.GratbotLogger import gprint

class PoseTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.pose=BayesianArray(np.zeros(3),np.array([[1e-6,0,0],[0,1e-6,0],[0,0,100]]))
        #self.velocity=BayesianArray(ndim=3)
        #self.last_timestamp=time.time()
        self.min_time_update=1e-4
        #regular kalman filters use a process variance that adds with time
        #I'm instead going to stick with a minimum variance
        #self.pose_min_covariance=np.array([0.05**2,0.05*0.05,0.05**2]) #5cm, 3 degrees
        self.pose_min_covariance=np.array([0.02**2,0.02*0.05,0.01**2]) #2cm, 1 degrees
        self.pose_how_often=0.1
        self.time_pose_record=[ [time.time(),self.pose] ]
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


    def get_keys(self):
        return [ "pose_measurement","pose_offset" ]

    def get_name(self):
        return "PoseTrackerGyrus"

    def update_with_pose_measurement(self,measurement_barray):
        #wrap the phase of the update to assume I turned the closest way
        if measurement_barray.vals[2]-self.pose.vals[2]>np.pi:
            measurement_barray.vals[2]=measurement_barray.vals[2]-2*np.pi
        if measurement_barray.vals[2]-self.pose.vals[2]<-np.pi:
            measurement_barray.vals[2]=measurement_barray.vals[2]+2*np.pi

        self.pose=self.pose.updated(measurement_barray)
        for i in range(3):
            if self.pose.covariance[i][i]<self.pose_min_covariance[i]:
                self.pose.covariance[i][i]=self.pose_min_covariance[i]
        #wrap the phase of my final position
        if self.pose.vals[2]>np.pi:
            self.pose.vals[2]-=2*np.pi
        if self.pose.vals[2]<-np.pi:
            self.pose.vals[2]+=2*np.pi

    #def update_with_velocity_measurement(measurement_barray):
    #    self.velocity=self.velocity.updated(measurement_barray)
    #    for i in range(3):
    #        if self.pose.covariance[i][i]<self.pose_min_covariance[i]:
    #            self.pose_covariance[i][i]=self.pose_min_covariance[i]

    #def update_with_velocity_offset(offset_barray):
    #    self.velocity.vals=self.velocity.vals+offset_barray.vals
    #    self.velocity.covariance=self.velocity.covariance+offset_barray.vals

    def get_pose_message(self,timestamp=None):
        if timestamp==None:
            timestamp=time.time()
        if timestamp-self.time_pose_record[-1][0]<self.pose_how_often:
            return
        #conditions for a pose to be considered stable
        #1)  hasn't changed in the last half second
        #2)  uncertainty is no more than 10 percent greater than min uncertainty
        self.time_pose_record.append([ timestamp,self.pose])
        while (timestamp-self.time_pose_record[0][0])>2: #remove anything beyond a full two second
            self.time_pose_record.pop(0)
        xs=[ x[1].vals[0] for x in self.time_pose_record]
        ys=[ x[1].vals[1] for x in self.time_pose_record]
        ts=[ x[1].vals[2] for x in self.time_pose_record]
        dx=np.max(xs)-np.min(xs)
        dy=np.max(ys)-np.min(ys)
        dt=np.max(ts)-np.min(ts)
        twodegrees=np.radians(2)
        if abs(dx)<0.02 and abs(dy)<0.02 and abs(dt)<twodegrees and np.sqrt(self.pose.covariance[2][2])<twodegrees:
            m={"timestamp": timestamp,"latest_pose": self.pose.to_object(),"pose_notes": "pose_is_stable"}
            self.broker.publish(m,"latest_pose")
        else:
            #gprint("pose not stable {} {} {} {}".format(abs(dx),abs(dy),abs(dt), np.sqrt(self.pose.covariance[2][2]) ))
            #gprint("vs not stable {} {} {} {}".format(0.02,0.02,twodegrees,twodegrees))
            #gprint("pose t is {}".format(self.pose.vals[2]))
            m= {"timestamp": timestamp,"latest_pose": self.pose.to_object(),"pose_notes": "none"}
            self.broker.publish(m,"latest_pose")

    def read_message(self,message):
        #TODO suppose I receive a pose measurement with a timestamp from before the last pose offset
        #should I be able to figure out how to update that measurement?
        if "pose_measurement" in message:
            update=BayesianArray.from_object(message["pose_measurement"])
            self.update_with_pose_measurement(update)
            #self.broker.publish(self.get_pose_message(timestamp=message["timestamp"]),"latest_pose")
            self.get_pose_message(timestamp=message["timestamp"])

        if "pose_offset" in message:
            update=BayesianArray.from_object(message["pose_offset"])
            self.pose.vals+=update.vals
            self.pose.covariance+=update.covariance
            self.get_pose_message(timestamp=message["timestamp"])
#        if "velocity_delta" in message:
#            update=BayesianArray.from_object(message["velocity_delta"])
#            self.update_with_velocity_offset(update)
#        if "update_velocity" in message:
#            update=BayesianArray.from_object(message["update_velocity"])
#            self.update_with_velocity_measurement(update)
#            ret.append(self.get_pose_message())
