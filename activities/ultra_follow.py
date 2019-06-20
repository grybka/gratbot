#Try to follow an object found with ultrasonics
#in particular, make sure the closest thing is not too far, not too close
import sys
import yaml
import time
import logging
import traceback
import numpy as np
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotMotor
root = logging.getLogger()
root.setLevel(logging.INFO)

def unicode_sparkline(data_array,minval,maxval):
    bins=np.linspace(minval,maxval,8,endpoint=False)
    vals=np.digitize(data_array,bins)
    ret=""
    for i in range(len(data_array)):
        if vals[i]<=1:
            ret+=' '
        else:
            if vals[i]>8:
                vals[i]=8
            ret+=chr(9600+vals[i])
    return ret

def ultra_scan(robot):
#make an ultrasonic measurement at several angles
#returns distance at each angle
    ultra_yaw_servo=robot["camera_yaw_servo"]
    angles=[-0.2,-0.1,0,0.1,0.2]
    response_dist=[]
    response_stdev=[]
    response_angle=[]
    for angle in angles:
        #move the head
        ultra_yaw_servo.setpos_fraction(angle);
        #let it settle
        time.sleep(0.1)
        #measure distance
        dist,stdev=robot["ultrasonic_sensor"].average_distance(10)
        response_angle.append(angle)
        response_dist.append(dist)
        response_stdev.append(stdev)
    ultra_yaw_servo.setpos_fraction(0);
    return response_angle,response_dist,response_stdev

        


#This is where our main function begins
if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    while True:
        angle,dist,stdev=ultra_scan(robot)
        toprint=unicode_sparkline(dist,0,1.0):
        print(toprint)
