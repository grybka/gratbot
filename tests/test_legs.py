
#A server that allows the bot to be controlled over tcp
import SocketServer
import yaml
import time
import logging
import sys
import json

sys.path.append('../hardware_interface')
import hardware
import leg_control

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../hardware_interface/hardware_config_with_sound.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])

while True:
	inp = input("ready to quit?")
	if inp="y":
		break
