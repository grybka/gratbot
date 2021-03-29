from Gyrus import ThreadedGyrus
import numpy as np
from uncertainties import ufloat
from underpinnings.BayesianArray import BayesianArray
import time

#1)  keep track of change in pose when motors are actvie
#2)  TODO periodically learn the expected velocity given motors
#3)  TODO report to the pose tracker what velocity to expect
#4)  TODO does this also handle requests to move a certain distance?



def fit_slope(xs,ys,sigmasquares,m_prior,m_prior_unc):
    #TODO it is trivial to include a prior on the slope here
    #given motion records and a single dimension, fit to y=m*x
    #returns best fit m, m_unc, chi-square/n
    xxsum=0
    xysum=0
    yysum=0
    prior_uncsquare=m_prior_unc**2
    #print("prior is {}".format(ufloat(m_prior,m_prior_unc)))

    for i in range(len(xs)):
        x=xs[i]
        y=ys[i]
        sigmasquare=sigmasquares[i]
        xxsum+=x*x/sigmasquare
        xysum+=x*y/sigmasquare
        yysum+=y*y/sigmasquare
    myslope=(xysum+m_prior/prior_uncsquare)/(xxsum+1/prior_uncsquare)
    #print("myslope is {}".format(myslope))
    myslope_unc=np.sqrt(1/(xxsum/sigmasquare+1/prior_uncsquare))
    #print("myslope  unc is {}".format(myslope_unc))
    chisq=myslope*myslope*xxsum-2*myslope*xysum+yysum
    chisq=chisq/len(xs)
    if chisq>1: #inflate uncertainties if I'm underestimating error bars on data
        myslope_unc*=np.sqrt(chisq)
    return myslope,myslope_unc,chisq


def get_rot_matrix2(theta):
    return np.array([[-np.sin(theta),np.cos(theta)],[np.cos(theta),np.sin(theta)]])

def get_rot_matrix3(theta):
    return np.array([[-np.sin(theta),np.cos(theta),0],
                     [np.cos(theta),np.sin(theta),0],
                     [0,0,1]])

class MotionEstimationGyrus(ThreadedGyrus):

    def __init__(self,broker):
        self.previous_motor_activity=[0,0,0]
        self.pose_at_last_motion_start=np.array([0,0,0])
        self.timestamp_at_last_motion_start=0
        self.last_motor_activity_timestamp=0
        self.last_pose=np.array([0,0,0])

        #local storage of [time,pose] for fitting of parameters
        self.pose_history=[]
        #local storage of [time,motor behavior] for fitting of parameters
        self.motor_history=[]
        self.last_history_extraction=0 #a timestamp
        self.teachable_moments=[]



        #self.motion_record=[] # ( change in pose, change in time, motor behavior)
        self.turn_record=[]
        self.ahead_record=[]

        self.turn_slope=0 #in units of meters per turn-magnitude-second
        self.turn_slope_unc=10 #for max uncertainty, at start
        self.min_fraction_turn_slope_unc=0.1
        self.ahead_slope=0
        self.ahead_slope_unc=1.0
        self.min_fraction_ahead_slope_unc=0.1
        super().__init__(broker)



    def save_config(self):
        #TODO save things other than just the motion record
        output_config={}
        my_config={}
        my_config["turn_slope"]=float(self.turn_slope)
        my_config["turn_slope_unc"]=float(self.turn_slope_unc)
        my_config["ahead_slope"]=float(self.ahead_slope)
        my_config["ahead_slope_unc"]=float(self.ahead_slope_unc)
        output_config["MotionGyrus"]=my_config
        return output_config

    def load_config(self,config):
        try:
            my_config=config["MotionGyrus"]
            self.turn_slope=my_config["turn_slope"]
            self.turn_slope_unc=my_config["turn_slope_unc"]
            self.ahead_slope=my_config["ahead_slope"]
            self.ahead_slope_unc=my_config["ahead_slope_unc"]
            print("MotionGyus loaded OK")
        except:
            print("Unable to load MotionGyrus info from file")


    def predict_pose_offset_turn(self,turn_power):
        predicted_theta_change=turn_power*self.turn_slope
        predicted_theta_change_unc=abs(turn_power*self.turn_slope_unc)
        return BayesianArray(np.array([0,0,predicted_theta_change]),np.array([ [0,0,0],[0,0,0],[0,0,predicted_theta_change_unc**2]]))

    def predict_pose_offset_ahead(self,ahead_power):
        predicted_ahead_change=ahead_power*self.ahead_slope
        predicted_ahead_change_unc=abs(ahead_power*self.ahead_slope_unc)
        pose_offset_ahead=BayesianArray(np.array([predicted_ahead_change,0,0]),np.array( [ [predicted_ahead_change_unc**2,0,0],
                                                               [0,0,0],
                                                               [0,0,0]]))
        pose_offset=pose_offset_ahead.applymatrix(get_rot_matrix3(self.last_pose[2]))
        return pose_offset


    def get_keys(self):
        return [ "clock_pulse","latest_pose","move_command","drive/motors_active" ]

    def get_name(self):
        return "MotionGyrus"

    def read_message(self,message):
        ret=[]
        if "latest_pose" in message:
            the_pose=BayesianArray.from_object(message["latest_pose"])
            self.pose_history.append([message["timestamp"],the_pose]) #save to recent history for fitting
            self.last_pose=np.array(message["latest_pose"]["vals"])
            self.last_pose_covariance=np.array(message["latest_pose"]["covariance"])
            #extract potential tecahable moments every few seconds
            check_every_n_seconds=2
            if message["timestamp"]>self.last_history_extraction+check_every_n_seconds:
                self.extract_recent_motor_induced_pose_changes()
                self.last_history_extraction=message["timestamp"]
                self.learn_if_you_can(0)
                self.learn_if_you_can(2)

        if "drive/motors_active" in message:
            motors_active=message["drive/motors_active"]
            self.motor_history.append([message["timestamp"],motors_active])
            self.previous_motor_activity=motors_active
            if motors_active!=[0,0,0]:
                if motors_active[2]!=0:
                    #print("predicting pose turn")
                    dt=message["timestamp"]-self.last_motor_activity_timestamp
                    turn_power=motors_active[2]*dt
                    pose_offset=self.predict_pose_offset_turn(turn_power)
                    self.broker.publish({"timestamp": time.time(),"pose_offset": pose_offset.to_object(),"notes": "turn prediction"},"pose_offset")
                if motors_active[0]!=0: #forward backward
                    #print("predicting pose ahead")
                    dt=message["timestamp"]-self.last_motor_activity_timestamp
                    #print("got an ahead message {}".format(message))
                    ahead_power=motors_active[0]*dt
                    pose_offset=self.predict_pose_offset_ahead(ahead_power)
                    self.broker.publish({"timestamp": time.time(),"pose_offset": pose_offset.to_object(),"notes": "ahead prediction"},"pose_offset")
            self.last_motor_activity_timestamp=message["timestamp"]
        if "move_command" in message:
            if message["move_command"]["type"]=="turn":
                angle=message["move_command"]["angle"]
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "turn","magnitude": angle/self.turn_slope}},"motor_command")
            if message["move_command"]["type"]=="ahead":
                print(message["move_command"])
                distance=message["move_command"]["distance"]
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "ahead","magnitude": distance/self.ahead_slope}},"motor_command")

    def learn_if_you_can(self,axis):
        min_teachable_size=2
        #self.teachable_moments.append({"delta_pose": elem[3],"delta_time": elem[1]-elem[0],"motor_command": elem[2]})
        to_use=[]
        xs=[]
        ys=[]
        sigmasquares=[]

        for i in range(len(self.teachable_moments)):
            t=self.teachable_moments[i]
            if t["motor_command"][axis]!=0: #
                to_use.append(i)
                ys.append(t["delta_pose"].vals[axis])
                xs.append(t["motor_command"][axis]*t["delta_time"])
                sigmasquares.append(t["delta_pose"].covariance[axis][axis])
        if len(xs)>=min_teachable_size:
            #print("learning from {} examples for axis {}".format(len(xs),axis))
            self.teachable_moments=[i for j, i in enumerate(self.teachable_moments) if j not in to_use]
            if axis==2: #turn
                #print("learning turns")
                m_prior=self.turn_slope
                m_prior_unc=self.turn_slope_unc
                #print("before turn slope {}".format(ufloat(self.turn_slope,self.turn_slope_unc)))
                self.turn_slope,self.turn_slope_unc,chisq=fit_slope(xs,ys,sigmasquares,m_prior,m_prior_unc)
                if self.turn_slope_unc/max(abs(self.turn_slope),1e-3)<self.min_fraction_turn_slope_unc:
                    self.turn_slope_unc=abs(self.min_fraction_turn_slope_unc*self.turn_slope)
                #print("after turn slope {}".format(ufloat(self.turn_slope,self.turn_slope_unc)))
            if axis==0: #ahead
                print(xs)
                print(ys)
                print("learning ahead")
                m_prior=self.ahead_slope
                m_prior_unc=self.ahead_slope_unc
                print("before ahead slope {}".format(ufloat(self.ahead_slope,self.ahead_slope_unc)))
                self.ahead_slope,self.ahead_slope_unc,chisq=fit_slope(xs,ys,sigmasquares,m_prior,m_prior_unc)
                if self.ahead_slope_unc/max(abs(self.ahead_slope),1e-3)<self.min_fraction_ahead_slope_unc:
                    self.ahead_slope_unc=abs(self.min_fraction_ahead_slope_unc*self.ahead_slope)
                print("after ahead slope {}".format(ufloat(self.ahead_slope,self.ahead_slope_unc)))
        else:
            #print("not enough examples to learn axis {}".format(axis))
            pass


    def extract_pose_history_between_times(self,start_time,stop_time):
        return list(filter(lambda x: x[0]>start_time and x[0]<stop_time,self.pose_history))

    def extract_recent_motor_induced_pose_changes(self):
        #print("motor history length {}".format(len(self.motor_history)))
        while True:
            elem=self.get_last_motor_induced_pose_changes()
            if elem==None:
                break
            else:
                delta_pose_xy=elem[4]-elem[3]
                delta_pose=delta_pose_xy.applymatrix(get_rot_matrix3(elem[3].vals[2]).T)
                #wrap delta_pose properly
                while delta_pose.vals[2]>np.pi:
                        delta_pose.vals[2]=delta_pose.vals[2]-2*np.pi
                while delta_pose.vals[2]<-np.pi:
                        delta_pose.vals[2]=delta_pose.vals[2]+2*np.pi
                #delta_pose.covariance[2][2]=100
                self.teachable_moments.append({"delta_pose": delta_pose,"delta_time": elem[1]-elem[0],"motor_command": elem[2]})
                while self.motor_history[0][0]<elem[1]:
                    self.motor_history.pop(0)
                while self.pose_history[0][0]<elem[1]:
                    self.pose_history.pop(0)
        #print("motor history length {}".format(len(self.motor_history)))
        #print("n teachable moments {}".format(len(self.teachable_moments)))

    def get_last_motor_induced_pose_changes(self):
        #for each motor on period
        #try to find the best pose measurement in between periods
        try:
            first_motor_usage=next(filter(lambda x: x[1]!=[0,0,0],self.motor_history))
        except:
            #print("no first motor usage found")
            return None
        motor_type=first_motor_usage[1]
        try:
            last_motor_usage=next(filter(lambda x: x[1]!=motor_type and x[0]>first_motor_usage[0],self.motor_history))
        except:
            #print("no last motor usage found")
            return None
        try:
            end_bound=next(filter(lambda x: x[1]!=[0,0,0] and x[0]>last_motor_usage[0],self.motor_history))
        except:
            end_bound=[time.time(),[0,0,0]]

        #print("times are {} to ({},{})".format(first_motor_usage[0],last_motor_usage[0],end_bound[0]))
        poses_before=self.extract_pose_history_between_times(0,first_motor_usage[0])
        poses_after=self.extract_pose_history_between_times(last_motor_usage[0],end_bound[0])
        if len(poses_before)==0:
            #print("no poses before")
            return None
        if len(poses_after)==0:
            #print("no poses after")
            return None
        best_before=poses_before[np.argmin([ np.trace(p[1].covariance) for p in poses_before ])][1]
        best_after=poses_after[np.argmin([ np.trace(p[1].covariance) for p in poses_after ])][1]

        #delta_pose=best_after-best_before
        #great, so I have a pose difference, and a motor time
        #return start time, end time, motor usage, and delta pose
        return [first_motor_usage[0],last_motor_usage[0],motor_type,best_before,best_after]
