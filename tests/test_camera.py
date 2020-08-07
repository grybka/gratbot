
import yaml
import time
import logging
import sys
import json
import cv2

sys.path.append('../hardware_interface')
import hardware

root = logging.getLogger()
root.setLevel(logging.INFO)

#initialize hardware
config_file=open("../hardware_interface/hardware_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])

time.sleep(2)
image=robot["camera"].acquire_image()
cv2.imwrite("camera_test.png",image)
