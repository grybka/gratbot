#Kalman Pose Filter
from gyrii.underpinnings.BayesianArray import BayesianArray
import numpy as np
from filterpy.common import kinematic_state_transition,Q_discrete_white_noise
from filterpy.kalman import predict, update
from gyrii.underpinnings.GratbotLogger import gprint

class KalmanPoseFilter:
    def __init__(self,x_unc,angle_unc,v_unc,vangle_unc):
        P=np.diag([x_unc**2,v_unc**2,x_unc**2,v_unc**2,angle_unc**2,vangle_unc**2])
        self.pose=BayesianArray(np.zeros(6),P) #(x,vx,y,vy,theta,vtheta)
        self.last_time_update=0
        self.q_variance=20.0 #TODO figure this out
        self.dt_threshhold=0.01

    def set_q_variance(self,num):
        self.q_variance=num

    def declare_position_uncertainty(self,x_unc,angle_unc):
        #note you lose velocity covariance, if that's a think
        P=np.diag([x_unc**2,self.pose.covariance[1][1],x_unc**2,self.pose.covariance[3][3],angle_unc**2,self.pose.covariance[5][5]])
        self.pose.covariance=P

    def get_xytheta(self):
        return BayesianArray(self.pose.vals[::2],self.pose.covariance[::2,::2])

    def get_vxvyvtheta(self):
        return BayesianArray(self.pose.vals[1::2],self.pose.covariance[1::2,1::2])

    def wrap_phase(self):
        self.pose.vals[4]=(self.pose.vals[4] + np.pi) % (2 * np.pi) - np.pi

    def predict_to_time(self,timestamp):
        dt=timestamp-self.last_time_update
        if dt<self.dt_threshhold:
            #gprint("skipping predict step")
            return #just don't bother for too short a time
        F=np.array([[1 ,dt,0 ,0 ,0 ,0],
                    [0 , 1,0 ,0 ,0 ,0],
                    [0 , 0,1 ,dt,0 ,0],
                    [0 , 0,0 , 1,0 ,0],
                    [0 , 0,0 , 0,1 ,dt],
                    [0 , 0,0 , 0,0 , 1]])
        Q=Q_discrete_white_noise(dim=2,block_size=3,dt=dt,var=self.q_variance)
        self.pose.vals,self.pose.covariance = predict(self.pose.vals,self.pose.covariance,F,Q)
        self.wrap_phase()
        self.last_time_update=timestamp

    def update_with_pose_measurement(self,timestamp,position): #bayesean array of x,y,theta
        self.predict_to_time(timestamp)
        z=position.vals
        delta=z[2]-self.pose.vals[4]
        newdelta=(delta + np.pi) % (2 * np.pi) - np.pi
        z[2]=newdelta+self.pose.vals[4]
        R=position.covariance
        H=np.array([ [1,0,0,0,0,0],
                     [0,0,1,0,0,0],
                     [0,0,0,0,1,0]])
        self.pose.vals,self.pose.covariance = update(self.pose.vals,self.pose.covariance, z, R, H)
        self.wrap_phase()

    def update_with_position_measurement(self,timestamp,position): #bayesean array of first x,y
        self.predict_to_time(timestamp)
        z=position.vals
        R=position.covariance
        H=np.array([ [1,0,0,0,0,0],
                     [0,0,1,0,0,0]])
        self.pose.vals,self.pose.covariance = update(self.pose.vals,self.pose.covariance, z, R, H)

    def update_with_velocity_measurement(self,timestamp,velocity): #bayesian array of vx,vy
        self.predict_to_time(timestamp)
        z=velocity.vals
        R=velocity.covariance
        H=np.array([ [0,1,0,0,0,0],
                     [0,0,0,1,0,0],
                     [0,0,0,0,0,1]])
        self.pose.vals,self.pose.covariance = update(self.pose.vals,self.pose.covariance, z, R, H)

    def update_with_angle_measurement(self,timestamp,angle,angle_unc):
        self.predict_to_time(self,timestamp)
        if self.pose[2]-angle > np.pi:
            angle+=2*np.pi
        if self.pose[2]-angle < -np.pi:
            angle-=2*n.pi
        z=np.array([angle])
        R=np.array([[angle_unc*angle_unc]])
        H=np.array([ [0,0,0,0,1,0]] )
        self.pose.vals,self.pose.covariance = update(self.pose.vals,self.pose.covariance, z, R, H)
        self.wrap_phase()
