"""Leg Controls"""
from hardware import GratbotSpimescape
from hardware import create_hardware_item
import threading
import time
import numpy as np
import logging
import base64
from hardware import _all_gratbot_spimescapes


class LegControl(GratbotSpimescape):
    # A class to control a hexapod
    # Forward speed is in  [-1,1] with negative being bacwards
    # Left right is in [-1,1] with 0 being straight

    def __init__(self, datastruct, hardware):
        self.hardware = hardware
        self.left_cadence = datastruct["left_cadence"]
        self.right_cadence = datastruct["right_cadence"]
        self.period_seconds = datastruct["period_seconds"]
        self.time_offset = time.time()
        self.left_right = 0
        self.forward_backward = 0
        self.thread = threading.Thread(target=self._daemon_loop)
        self.thread.daemon = True
        self.thread_should_quit = False
        self.thread.start()

    def set_forward_backward(self, fb):  # +1 is forward, -1 is backward
        self.forward_backward = np.clip(fb, -1, 1)

    def set_left_right(self, lr):  # +1 is right, -1 is left
        self.left_right = np.clip(fb, -1, 1)

    def _daemon_loop(self):
        while not self.thread_should_quit:
            # note, fifty times a second seemed to freeze
            time.sleep(0.04)  # check twenty-five times a second
            self.update_servo_positions()

    def update_servo_positions(self):
        now = time.time()
        t = now-self.time_offset
        phase = (t % self.period_seconds)/self.period_seconds
        # fb controls magnitude
        # lr controls relative phase
        # TODO maybe I need to multiply lr by sign of fb to make sense
        left_phase = (phase+0.5*self.left_right) % 1.0
        for motor in self.right_cadence:
            next_pos = self.forward_backward*self.get_motor_pos(phase, self.right_cadence[motor]))
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)
        for motor in self.left_cadence:
            next_pos=self.forward_backward*self.get_motor_pos(left_phase, self.left_cadence[motor]))
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)

    def set(self, endpoint, value):
        if endpoint == "left_right":
            self.set_left_right(value)
            return
        if endpoint == "forward_backward":
            self.set_forward_backward(value)
            return
        raise Exception("unknown endpoint {}".format(endpoint))

    def get(self, endpoint):
        if endpoint == "left_right":
            return self.set_left_right
        if endpoint == "forward_backward":
            self.set_forward_backward(value)
            return self.set_forward_backward
        raise Exception("unknown endpoint {}".format(endpoint))

_all_gratbot_spimescapes["LegControl"]=LegControl

class CadenceLegControl(GratbotSpimescape):
    # controls cadence of leg motions
    # should have the feature "cadences"
    # period_seconds: 10
    # cadences:
    #   walking:
    #     left_front:
    #         times: [0,0.5,.9]
    #         positions: [300,350,325]
    def __init__(self, datastruct, hardware):
        self.cadences=datastruct["cadences"]
        self.hardware=hardware
        self.on_cadence="none"
        self.period_seconds=datastruct["period_seconds"]
        self.time_offset=time.time()
        self.thread=threading.Thread(target=self._daemon_loop)
        self.thread.daemon=True
        self.thread_should_quit=False
        self.thread.start()

    def _daemon_loop(self):
        while not self.thread_should_quit:
            # note, fifty times a second seemed to freeze
            time.sleep(0.04)  # check twenty-five times a second
            self.update_servo_positions()

    def update_servo_positions(self):
        now=time.time()
        t=now-self.time_offset
        t_cycle=(t % self.period_seconds)/self.period_seconds
        # logging.info("time {}".format(t_cycle))
        if self.on_cadence not in self.cadences:
            return  # do nothing
        cad=self.cadences[self.on_cadence]
        for motor in cad:
            new_pos=np.interp(
                t_cycle, cad[motor]["times"], cad[motor]["positions"])
            self.hardware[motor].setpos_fraction(new_pos)
        time.sleep(0.001)

    def set(self, endpoint, value):
        raise Exception("set unhandled")
    def get(self, endpoint):
        raise Exception("get unhandled")
_all_gratbot_spimescapes["CadenceLegControl"]=CadenceLegControl
