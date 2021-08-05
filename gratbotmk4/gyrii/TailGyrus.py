
from Gyrus import ThreadedGyrus
import numpy as np
import logging
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class TailGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.how_often=0.2
        self.next_act=0

    def get_keys(self):
        return ["clock_pulse","servo_response","gyrus_config"]

    def get_name(self):
        return "TailGyrus"

    def read_message(self,message):
        if "gyrus_config" in message:
            if message["gyrus_config"]["target_gyrus"]=="TailGyrus":
                ...
        if "clock_pulse" in message:
            if time.time()>self.next_act:
                self.next_act=time.time()+self.how_often

                freq=np.pi
                angle=90+40*np.sin(message["timestamp"]*freq)

                servo_command={"timestamp": time.time(),"servo_command": {"servo_number":1,"angle": angle}}
                self.broker.publish(servo_command,"servo_command")
