
import yaml
import time
import logging
import sys
import json

sys.path.append('../hardware_interface')
import hardware
#import leg_control

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../hardware_interface/hardware_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])

print("forward")
robot["wheel_motor"].go(1,100)
time.sleep(5)
print("pause")
robot["wheel_motor"].go(1,0)
time.sleep(1)
print("backward")
robot["wheel_motor"].go(1,-100)
time.sleep(5)
print("stop")
robot["wheel_motor"].stop()
