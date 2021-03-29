
from Gyrus import ThreadedGyrus
from GratbotCommsMk2 import GratbotCommsMk2
import numpy as np
import logging
import time


class CommsGyrus(ThreadedGyrus):
    def __init__(self,broker,simulation_mode=False):
        self.turn_speed=0.6
        self.fb_speed=0.6
        self.simulation_mode=simulation_mode
        if not simulation_mode:
            logging.info("Connecting to Gratbot comms")
            self.gratbot_comms = GratbotCommsMk2("10.0.0.4", 9999)
            self.gratbot_comms.set(["ultrasonic_sensor","update_frequency"],4)
        super().__init__(broker)

    def get_keys(self):
        return [ "clock_pulse","motor_command" ]

    def get_name(self):
        return "CommsUpdateGyrus"

    def read_message(self,message):
        if "clock_pulse" in message:
            if not self.simulation_mode:
                comms_message=self.gratbot_comms.update()
                my_keys=[]
                for key in comms_message:
                    if key != "timestamp":
                        my_keys.append(key)
                comms_message["timestamp"]=time.time() #in case its clock has skew
                self.broker.publish(comms_message,my_keys)
            else: #SIMULATION MODE
                self.broker.publish({"timestamp": time.time(),"magnetometer/b_field": [0,0,0]},"magnetometer/b_field")
        if "motor_command" in message and not self.simulation_mode:
            if message["motor_command"]["type"]=="turn":
                duration=message["motor_command"]["magnitude"]
                self.gratbot_comms.set(["drive","translate"],[0,0,np.sign(duration)*self.turn_speed,abs(duration)])
            if message["motor_command"]["type"]=="ahead":
                duration=message["motor_command"]["magnitude"]
                self.gratbot_comms.set(["drive","translate"],[np.sign(duration)*self.fb_speed,0,0,abs(duration)])
