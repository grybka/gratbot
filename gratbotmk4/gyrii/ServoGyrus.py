
#motor gyrus
import threading
import logging
import board
import time
import numpy as np
from Gyrus import ThreadedGyrus
from adafruit_servokit import ServoKit

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class ServoGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.kit = ServoKit(channels=16)
        super().__init__(broker)

    def get_keys(self):
        return ["servo_command"]

    def get_name(self):
        return "ServoGyrus"

    def read_message(self,message):
        if "servo_command" in message:
            m=message["servo_command"]
            now=time.time()
            servo_num=int(m["servo_number"])
            angle=m["angle"]
            self.kit.servo[0].angle=angle
            self.broker.publish({"timestamp": time.time(),"servo_response": m},["motor_response"])
