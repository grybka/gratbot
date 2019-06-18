import sys
import yaml
import time
import logging
import traceback
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotLED

root = logging.getLogger()
root.setLevel(logging.INFO)

#This is where our main function begins
if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    colors=[GratbotLED.red,
             GratbotLED.blue,
             GratbotLED.green,
             GratbotLED.pink,
             GratbotLED.cyan,
             GratbotLED.white,
             GratbotLED.yellow,
             GratbotLED.off]
    names=["red",
           "blue",
           "green",
           "pink",
           "cyan",
           "white",
           "yellow",
           "off"]
    for i in range(len(colors)):
        print("color {}".format(names[i]))
        x=colors[i]
        robot["left_front_led"].set_color(x)
        robot["led_strip"].set_all_color(Color(*colors[i]))
        time.sleep(1)



