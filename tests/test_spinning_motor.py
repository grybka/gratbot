
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
for key in robot:
    print("{}".format(key))

print("forward")
magnitude=40.0
robot["wheel_motor"].go(hardware.GratbotMotor.forward,magnitude)
time.sleep(3)
print("pause")
robot["wheel_motor"].go(hardware.GratbotMotor.forward,0)
time.sleep(1)
print("backward")
robot["wheel_motor"].go(hardware.GratbotMotor.backward,magnitude)
time.sleep(3)
print("stop")
robot["wheel_motor"].stop()
