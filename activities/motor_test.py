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

#last result
#100 was fast
#80 was fine
#60 was slow
#40 barely moved
#20 did not move

#This is where our main function begins
if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    robot["wheel_motor"].stop()
    speeds=[100,80,60,40,20,0]
    for s in speeds:
        logging.info("Forward {}".format(s))
        robot["wheel_motor"].go(GratbotMotor.forward,s)
        time.sleep(2)
        logging.info("Backward {}".format(s))
        robot["wheel_motor"].go(GratbotMotor.backward,s)
        time.sleep(2)
    robot["wheel_motor"].stop()
