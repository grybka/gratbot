import sys
sys.path.append('../hardware_interface')
import yaml
import logging
import hardware
import leg_control

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../hardware_interface/mecanum_hardware_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])
robot_thread=hardware.GratbotHardwareThread(robot)

while True:
    a=input("forward_backward?")
    if a=="q":
        break
    b=input("side_sid?")
    c=input("turn?")
    robot["drive"].set("translate",[float(a),float(b),float(c)])
robot["drive"].set("front_right",0)
robot["drive"].set("front_left",0)
robot["drive"].set("back_right",0)
robot["drive"].set("back_left",0)
