from Gyrus import ThreadedGyrus
import numpy as np
import time
from gyrii.underpinnings.GratbotLogger import gprint

class Compass(ThreadedGyrus):
    def __init__(self,broker):
        #self.b_field_correction=np.array([-5.63,119.74,-58.58]) #new position
        self.b_field_correction=np.array([2.78,123.61,-58.58]) #new position
        self.expected_b_field_magnitude=50
        self.b_field_cutoff=9
        self.nonlinear_term=-0.446
        self.heading_offset=1.4 #radians
        #self.compass_uncertainty=10*2*np.pi/360 #10 degrees
        #self.compass_uncertainty=20*2*np.pi/360 #20 degrees
        #self.compass_uncertainty=0.4 #about 17 degrees
        #self.compass_uncertainty=0.04 #about 2 degrees
        self.compass_uncertainty=0.2 #about 10 degrees
        self.compass_dead_after_motor_start=0.1#time over which compass should not read after motor start
        self.last_motor_reading=0
        self.last_motor_start_time=0
        self.big_covariance=1e12
        self.last_compass_report=0
        self.compass_report_period=0.1
        self.recent_field_reading_x=[]
        self.recent_field_reading_y=[]
        self.recent_field_reading_z=[]
        #self.compass_disabled=False
        self.compass_disabled=True
        super().__init__(broker)
        #TODO should I not trigger compass readings quite so often

    def save_config(self):
        output_config={}
        myconfig={}
        myconfig["b_field_correction"]=self.b_field_correction.tolist()
        myconfig["heading_offset"]=self.heading_offset
        myconfig["nonlinear_term"]=self.nonlinear_term
        output_config["Compass"]=myconfig
        return output_config

    def load_config(self,config):
        try:
            myconfig=config["Compass"]
            self.b_field_correction=np.array(myconfig["b_field_correction"])
            self.heading_offset=myconfig["heading_offset"]
            self.nonlinear_term=myconfig["nonlinear_term"]
        except:
            print("Failed to load compass config")

    def get_keys(self):
        return [ "magnetometer/b_field","compass" ]

    def get_name(self):
        return "CompassGyrus"

    def read_message(self,message):
        #if "drive/motors_active" in message:
        #    if message["drive/motors_active"]!=self.last_motor_reading:
        #        self.last_motor_start_time=message["timestamp"]
        #    self.last_motor_reading=message["drive/motors_active"]
        if "compass" in message:
            gprint("received compass message {}".format(message["compass"]))
            if message["compass"]=="off":
                gprint("Turning off compass")
                self.compass_disabled=True
            if message["compass"]=="on":
                gprint("Turning on compass")
                self.compass_disabled=False
        if "magnetometer/b_field" in message:
            self.recent_field_reading_x.append(message["magnetometer/b_field"][0])
            self.recent_field_reading_y.append(message["magnetometer/b_field"][1])
            self.recent_field_reading_z.append(message["magnetometer/b_field"][2])
            last_n=3
            if len(self.recent_field_reading_x)<last_n:
                return
            while len(self.recent_field_reading_x)>last_n:
                self.recent_field_reading_x.pop(0)
                self.recent_field_reading_y.pop(0)
                self.recent_field_reading_z.pop(0)
            current_reading=np.array([ np.median(self.recent_field_reading_x), np.median(self.recent_field_reading_y),np.median(self.recent_field_reading_z)])
            if time.time()<self.last_compass_report+self.compass_report_period:
                return #don't over report the compass
            self.last_compass_report=time.time()
            b=current_reading-self.b_field_correction #corrected b field
            magnitude_error=abs(np.linalg.norm(b)-self.expected_b_field_magnitude)
            if magnitude_error>self.b_field_cutoff: #if I'm getting magnetized by something, ignore it
                return
            compass_heading_uncorrected=-np.arctan2(b[0],b[1])
            compass_heading=compass_heading_uncorrected+self.nonlinear_term*np.sin(compass_heading_uncorrected)-self.heading_offset
            #wrap the angle to -pi,pi
            if compass_heading<-np.pi:
                compass_heading=compass_heading+2*np.pi
            if compass_heading>np.pi:
                compass_heading=compass_heading-2*np.pi

            #if message["timestamp"]-self.last_motor_start_time<self.compass_dead_after_motor_start:
            #    return

            #for measuring pose
            pose_measurement_vals=[0,0,compass_heading]
            pose_measurement_covariance=[ [self.big_covariance,0,0],
                                         [0,self.big_covariance,0],
                                         [0,0,self.compass_uncertainty**2]]

            if not self.compass_disabled:
                self.broker.publish({"pose_measurement": {"vals":pose_measurement_vals,"covariance":pose_measurement_covariance},"timestamp": message["timestamp"],"notes": "compass"},"pose_measurement")
