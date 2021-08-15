
from Gyrus import ThreadedGyrus
import numpy as np
import logging
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class TailGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.tail_mood="impassive"
        self.how_often=0.1
        self.last_act=0
        #
        self.high_low_servo_num=2
        self.side_side_servo_num=1
        #for wagging
        self.wag_phase=0
        self.wag_frequency=2*np.pi
        self.wag_amplitude=60

    def get_keys(self):
        return ["clock_pulse","servo_response","gyrus_config"]

    def get_name(self):
        return "TailGyrus"

    def set_tail_high(self):
        logger.debug("Tail HIGH")
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number":self.high_low_servo_num,"angle": 180}}
        self.broker.publish(servo_command,"servo_command")

    def set_tail_low(self):
        logger.debug("Tail LOW")
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number":self.high_low_servo_num,"angle": 110}}
        self.broker.publish(servo_command,"servo_command")

    def wag_tail(self,dt):
        logger.debug("Tail WAG")
        self.wag_phase+=self.wag_frequency*dt
        angle=90+self.wag_amplitude*np.sin(self.wag_phase)
        servo_command={"timestamp": time.time(),"servo_command": {"servo_number":self.side_side_servo_num,"angle": angle}}
        self.broker.publish(servo_command,"servo_command")

    def read_message(self,message):
        if "gyrus_config" in message:
            if message["gyrus_config"]["target_gyrus"]=="TailGyrus":
                if "tail_mood" in message["gyrus_config"]:
                    self.tail_mood=message["gyrus_config"]["tail_mood"]
        if "clock_pulse" in message:
            dt=time.time()-self.last_act
            if dt<self.how_often:
                return
            self.last_act=time.time()
            if self.tail_mood=="anticipation":
                self.set_tail_high()
            if self.tail_mood=="happiness":
                self.set_tail_high()
                self.wag_tail(dt)
            if self.tail_mood=="disappointment":
                self.set_tail_low()
