import yaml
import logging
import sys
import hardware
import leg_control
sys.path.append('../hardware_interface')

root = logging.getLogger()
root.setLevel(logging.INFO)


#initialize hardware
config_file=open("../hardware_interface/raspclaws_config.yaml","r")
config_data=yaml.safe_load(config_file)
config_file.close()
robot=hardware.create_hardware(config_data["hardware"])
#robot["leg_controller"].on_cadence="tapping"
#robot["leg_controller"].on_cadence="walking"
robot["leg_controller"].on_cadence="walking_turn_left"

while True:
    fb = raw_input("Forward Backward?")
    if fb=="q":
        break
    lr = raw_input("Left Right?")
    if lr=="q":
        break
    robot["leg_controller"].set("forward_backward", fb)
    robot["leg_controller"].set("left_right", lr)
