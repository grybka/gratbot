#Try to follow an object found with ultrasonics
#in particular, make sure the closest thing is not too far, not too close
import sys
import yaml
import time
import logging
import traceback
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotMotor
root = logging.getLogger()
root.setLevel(logging.INFO)

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
        out_str1=""
        out_str2=""
        for i in range(len(dist)):
            out_str1+=" {}".format(dist[i])
            out_str2+=" {}".format(stdev[i])
            print(out_str1+"  |  "+out_str2)
        time.sleep(1)

