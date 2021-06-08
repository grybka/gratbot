import sys
sys.path.append('../gratbot_server_refactor/hardware')
import yaml
import logging
#import hardware
#import leg_control
from GratbotSpimescape import create_hardware
import CaterpillarDrive

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../gratbot_server_refactor/config/caterpillar_hardware_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=create_hardware(config_data["hardware"])

while True:
    a=input("left drive?")
    if a=="q":
        break
    b=input("right drive?")
    if b=="q":
        break
    c=input("duration?")
    if c=="q":
        break
    robot["drive"].set("go",[float(a),float(b),float(c)])
