import threading
import logging
import time
import numpy as np
from underpinnings.apa102 import APA102
from gpiozero import LED
from Gyrus import ThreadedGyrus
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class LEDDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.PIXELS_N=12
        self.dev=APA102(num_led=self.PIXELS_N)
        self.power=LED(5)
        self.power.on()

        self.last_trigger=0
        self.on_pixel=0

    def get_keys(self):
        return ["led_command","clock_pulse"]

    def get_name(self):
        return "LEDDisplayGyrus"

    def read_message(self,message):
        if "led_command" in message:
            m=message["led_command"]
        if "clock_pulse" in message:
            if time.time()<self.last_trigger+1:
                return
            self.last_trigger=time.time()
            self.on_pixel=(self.on_pixel+1)%self.PIXELS_N
            for i in range(self.PIXELS_N):
                if i==self.on_pixel:
                    self.dev.set_pixel(i,100,100,100)
                else:
                    self.dev.set_pixel(i,0,0,0)
            self.dev.show()
