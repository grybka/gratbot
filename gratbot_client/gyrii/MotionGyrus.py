from Gyrus import ThreadedGyrus
import numpy as np
from uncertainties import ufloat
from underpinnings.BayesianArray import BayesianArray
import time

from gyrii.underpinnings.GratbotLogger import gprint
#1)  keep track of change in pose when motors are actvie
#2)  TODO periodically learn the expected velocity given motors
#3)  TODO report to the pose tracker what velocity to expect
#4)  TODO does this also handle requests to move a certain distance?



def fit_slope(xs,ys,sigmasquares,m_prior,m_prior_unc):
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
    return np.array([[np.sin(cos),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])

def get_rot_matrix3(theta):
    return np.array([[np.cos(theta),np.sin(theta),0],
                     [-np.sin(theta),np.cos(theta),0],
                     [0,0,1]])

class MotionEstimationGyrus(ThreadedGyrus):

    def __init__(self,broker):

        self.motor_off_covariance=1e-4*np.eye(3)
        self.motor_on_min_covariance=10.0*np.eye(3)

        #self.offset_slope_matrix=np.array([[0.13,0.45,-0.017],
        #                                   [0.45,0,0],
        #                                   [0,0,-2.63]])
        #I multiply this matrix by my motor activation and get a speed
        self.motor_to_velocity=np.array([[-0.01090508,  0.42752218, -0.03158485],
                                [ 0.40995484,  0.01007396,  0.0637292 ],
                                [-0.03484511, -0.0503116 , -2.93494427]])
        #self.offset_slope_matrix_unc=self.offset_slope_matrix*0.2




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

        super().__init__(broker)



    def save_config(self):
        #TODO save things other than just the motion record
        output_config={}
        my_config={}
        output_config["MotionGyrus"]=my_config
        return output_config

    def load_config(self,config):
        try:
            my_config=config["MotionGyrus"]
            print("MotionGyus loaded OK")
        except:
            print("Unable to load MotionGyrus info from file")

    def get_keys(self):
        return [ "clock_pulse","latest_pose","move_command","drive/motors_active" ]

    def get_name(self):
        return "MotionGyrus"

    def motor_vetor_to_velocity(self,motor_vector):
            #velocity_vector=np.dot(self.motor_to_velocity,np.dot(get_rot_matrix3(self.last_pose[2]).T,motor_vector))
            velocity_vector=np.dot(get_rot_matrix3(self.last_pose[2]),np.dot(self.motor_to_velocity,motor_vector))
            velocity_vector_covariance=np.diag(np.array([0.08**2,0.08**2,0.1**2])) #this is hardcoded.
            return BayesianArray(velocity_vector,velocity_vector_covariance)

    def read_message(self,message):
        ret=[]
        if "latest_pose" in message:
            the_pose=BayesianArray.from_object(message["latest_pose"])
            self.last_pose=np.array(message["latest_pose"]["vals"])
            self.last_pose_covariance=np.array(message["latest_pose"]["covariance"])

        if "drive/motors_active" in message:
            motors_active=message["drive/motors_active"][0:3]
            duration=message["drive/motors_active"][3]
            self.motor_history.append([message["timestamp"],motors_active])
            self.previous_motor_activity=motors_active
            motor_vector=np.array(motors_active)
            if motors_active[0] !=0 or motors_active[1] !=0 or motors_active[2] !=0:
                self.broker.publish({"timestamp": time.time(),"pose_is_changing": True},["pose_is_changing"])
                #velocity_vector=np.dot(self.motor_to_velocity,np.dot(get_rot_matrix3(self.last_pose[2]),motor_vector))
                #velocity_vector_covariance=np.diag(np.array([0.08**2,0.08**2,0.1**2])) #this is hardcoded.
                #vel=BayesianArray(velocity_vector,velocity_vector_covariance)
                vel=self.motor_vetor_to_velocity(motor_vector)
            else:
                vel=BayesianArray(np.zeros(3),self.motor_off_covariance)
            #    self.broker.publish({"timestamp": time.time(),"pose_is_changing": True,"pose_certainty_lost": True},["pose_certainty_lost","pose_is_changing"])

            self.broker.publish({"timestamp": time.time(),"velocity_measurement": vel.to_object(),"notes": "motors active"},"velocity_measurement")
            self.last_motor_activity_timestamp=message["timestamp"]
        if "move_command" in message:
            speed=0.6 #this is a hack because I'm not consistent about when I have to do this
            if message["move_command"]["type"]=="turn":
                angle=message["move_command"]["angle"]
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "turn","magnitude": angle/(speed*self.motor_to_velocity[2][2])}},"motor_command")
            if message["move_command"]["type"]=="ahead":
                distance=message["move_command"]["distance"]
                #gprint("conversion factor is {}".format(self.offset_slope_matrix[0][1]))
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "ahead","magnitude": distance/(speed*self.motor_to_velocity[0][1])}},"motor_command")
            if message["move_command"]["type"]=="slew":
                distance=message["move_command"]["distance"]
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "slew","magnitude": distance/(speed*self.motor_to_velocity[1][0])}},"motor_command")
            if message["move_command"]["type"]=="complex":
                gprint("complex move command")
                dist3vector=np.array(message["move_command"]["distance"])
                motor3vector=np.dot(np.linalg.pinv(self.motor_to_velocity),dist3vector)
                maxval=np.max(abs(motor3vector))
                motor3vector*=speed/maxval
                duration=maxval/speed
                gprint("{} and {}".format(motor3vector,duration))
                if duration>4.0:
                    gprint("Aborting motion, it requests too long a duration {}".format(duration))
                    return
                motor4vector=[ motor3vector[0],motor3vector[1],motor3vector[2],duration]
                self.broker.publish({"timestamp": time.time(),"motor_command": {"type": "translate","vector": motor4vector}},"motor_command")
