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



def forward_until_wall(robot):
#move forward until you see a wall nearby
    motor_speed=0.6
    #point straight ahead
    robot["wheel_turn_servo"].setpos_fraction(0)
    #turn on motor
    robot["wheel_motor"].go(GratbotMotor.forward,motor_speed)
    #poll ultrasonics until wall is close
    while True:
        dist,stdev=robot["ultrasonic_sensor"].average_distance(0.1)
        if dist<0.5:
            robot["wheel_motor"].stop()
            return

def reverse_turn(left_not_right,time_seconds):
#turn and reverse 
    motor_speed=0.6
    #turn wheels to the right
    robot["wheel_turn_servo"].setpos_fraction(0.8)
    #turn on motor
    robot["wheel_motor"].go(GratbotMotor.backward,motor_speed)
    #wait a bit
    time.sleep(time_seconds)
    #stop and return
    robot["wheel_motor"].stop()

if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    try:
        while True:
            #move forward until I hit something
            forward_until_wall(robot)
            #turn and try again
            reverse_turn(True,0.5):
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)

    except Exception as e:
        logging.warning("Exception: {}".format(str(e)))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        traceback.print_tb(exc_tb)
        logging.warning("Type: {}".format(exc_type))
        logging.warning("File: {}".format(exc_tb.tb_frame.f_code.co_filename))
        logging.warning("Line: {}".format(exc_tb.tb_lineno))

        logging.warning("Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)

 

"""
class State:
    def __init__(self,robot):
        self.robot=robot

    def run_state(self):
        raise Exception("not implemented")
    def next_state(self):
        raise Exception("not implemented")

class ForwardState:
    def run_state(self):
        slow_speed=50
        self.robot["wheel_motor"].go("GratbotMotor.forward",slow_speed)
"""
