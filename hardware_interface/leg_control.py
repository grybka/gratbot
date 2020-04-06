from hardware import GratbotSpimescape
from hardware import create_hardware_item
import threading
import time
import numpy as np
import logging
import base64
from hardware import _all_gratbot_spimescapes

class LegControl(GratbotSpimescape):
    # controls cadence of leg motions
    # should have the feature "cadences"
    # period_seconds: 10
    # cadences:
    #   walking:
    #     left_front:
    #         times: [0,0.5,.9]
    #         positions: [300,350,325]
    def __init__(self, datastruct, hardware):
        self.cadences = datastruct["cadences"]
        self.hardware = hardware
        self.on_cadence = "none"
        self.period_seconds = datastruct["period_seconds"]
        self.time_offset = time.time()
        self.thread = threading.Thread(target=self._daemon_loop)
        self.thread.daemon = True
        self.thread_should_quit = False
        self.thread.start()

    def _daemon_loop(self):
        while not self.thread_should_quit:
            time.sleep(0.02)  # check fifty times a second
            self.update_servo_positions()

    def update_servo_positions(self):
        now = time.time()
        t = now-self.time_offset
        t_cycle = (t % self.period_seconds)/self.period_seconds
        #logging.info("time {}".format(t_cycle))
        if self.on_cadence not in self.cadences:
            return  # do nothing
        cad = self.cadences[self.on_cadence]
        for motor in cad:
            new_pos = np.interp(t_cycle, cad[motor]["times"], cad[motor]["positions"])
            self.hardware[motor].setpos_fraction(new_pos)
            #logging.info("moving {} to {}".format(motor,new_pos))

    def set(self, endpoint,value):
        raise Exception("set unhandled")
    def get(self, endpoint):
        raise Exception("get unhandled")
_all_gratbot_spimescapes["LegControl"] = LegControl
