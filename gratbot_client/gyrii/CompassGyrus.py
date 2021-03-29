from Gyrus import ThreadedGyrus
import numpy as np
import time

class Compass(ThreadedGyrus):
    def __init__(self,broker):
        self.b_field_correction=np.array([-5.63,119.74,-58.58]) #new position
        self.heading_offset=1.4 #radians
        self.compass_uncertainty=10*2*np.pi/360 #10 degrees
        self.compass_dead_after_motor_start=0.1#time over which compass should not read after motor start
        self.last_motor_reading=0
        self.last_motor_start_time=0
        self.big_covariance=1e12
        self.last_compass_report=0
        self.compass_report_period=0.1
        super().__init__(broker)
        #TODO should I not trigger compass readings quite so often

    def save_config(self):
        output_config={}
        myconfig={}
        myconfig["b_field_correction"]=self.b_field_correction.tolist()
        myconfig["heading_offset"]=self.heading_offset
        output_config["Compass"]=myconfig
        return output_config

    def load_config(self,config):
        try:
            myconfig=config["Compass"]
            self.b_field_correction=np.array(myconfig["b_field_correction"])
            self.heading_offset=myconfig["heading_offset"]
        except:
            print("Failed to load compass config")

    def get_keys(self):
        return [ "magnetometer/b_field" ]

    def get_name(self):
        return "CompassGyrus"

    def read_message(self,message):
        #if "drive/motors_active" in message:
        #    if message["drive/motors_active"]!=self.last_motor_reading:
        #        self.last_motor_start_time=message["timestamp"]
        #    self.last_motor_reading=message["drive/motors_active"]
        if "magnetometer/b_field" in message:
            if time.time()<self.last_compass_report+self.compass_report_period:
                return #don't over report the compass
            b=np.array(message["magnetometer/b_field"])-self.b_field_correction #corrected b field
            compass_heading=-np.arctan2(b[0],b[1])-self.heading_offset
            #wrap the angle to -pi,pi
            if compass_heading<-np.pi:
                compass_heading=compass_heading+2*np.pi
            if compass_heading>np.pi:
                compass_heading=compass_heading-2*np.pi

            if message["timestamp"]-self.last_motor_start_time<self.compass_dead_after_motor_start:
                return

            #for measuring pose
            pose_measurement_vals=[0,0,compass_heading]
            pose_measurement_covariance=[ [self.big_covariance,0,0],
                                         [0,self.big_covariance,0],
                                         [0,0,self.compass_uncertainty**2]]

            self.broker.publish({"pose_measurement": {"vals":pose_measurement_vals,"covariance":pose_measurement_covariance},"timestamp": message["timestamp"],"notes": "compass"},"pose_measurement")
