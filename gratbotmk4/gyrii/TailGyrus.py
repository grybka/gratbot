
from Gyrus import ThreadedGyrus
import numpy as np
import logging
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class TailGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)

    def get_keys(self):
        return ["clock_pulse","servo_response","gyrus_config"]

    def get_name(self):
        return "TailGyrus"

    def read_message(self,message):
        if "clock_pulse" in message:
            freq=0.1
            angle=90+10*np.sin(message["timestamp"]*freq)

            servo_command={"timestamp": time.time(),"servo_command": {"servo_number":1,"angle": angle}}
            self.broker.publish(servo_command,"servo_command")
