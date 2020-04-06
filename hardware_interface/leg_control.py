from hardware import GratbotSpimescape
from hardware import create_hardware_item
import threading
import time
import logging
import base64

class LegControl(GratbotSpimescape):
    #controls cadence of leg motions
    def __init__(self,datastruct):
        self.time_offset=time.time()
        self.thread=threading.Thread(target=self._daemon_loop)
        self.thread.daemon=True
        self.thread_should_quit=False
        self.thread.start()

    def _daemon_loop(self):
        while not self.thread_should_quit:
            time.sleep(0.1) #check ten times a second
            self.update_servo_positions()

    def update_servo_positions(self):
        now=time.time()
        t=now-self.time_offset
#how do I define where leg positons are supposed tobe as a function of time?

    def get_leg_pos(self,t,left_not_right,which_leg):
