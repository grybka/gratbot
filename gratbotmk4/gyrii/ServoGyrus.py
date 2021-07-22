
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
        self.max_angle=165
        self.min_angle=60
        super().__init__(broker)

    def get_keys(self):
        return ["servo_command"]

    def get_name(self):
        return "ServoGyrus"

    def set_servo_angle(self,servo_num,angle):
        m={"servo_number": servo_num,"angle": angle}
        angle=np.clip(angle,self.min_angle,self.max_angle)
        self.kit.servo[0].angle=angle
        self.broker.publish({"timestamp": time.time(),"servo_response": m},["servo_response"])

    def read_message(self,message):
        if "servo_command" in message:
#            logger.debug("servo command received")
            m=message["servo_command"]
            now=time.time()
            servo_num=int(m["servo_number"])
            if "angle" in m:
                angle=m["angle"]
                self.set_servo_angle(servo_num,angle)
            elif "delta_angle" in m:
#                logger.debug("delta angle {} command received".format(m["delta_angle"]))
                delta_angle=m["delta_angle"]
                angle=self.kit.servo[0].angle+m["delta_angle"]
                self.set_servo_angle(servo_num,angle)
