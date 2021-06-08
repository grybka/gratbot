import sys
sys.path.append('../hardware_interface')
import yaml
import logging
import hardware
import leg_control

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../hardware_interface/caterpillar_hardware_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])
robot_thread=hardware.GratbotHardwareThread(robot)

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
