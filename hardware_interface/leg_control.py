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
        self.left_hip_cadence = datastruct["left_hip_cadence"]
        self.left_knee_cadence = datastruct["left_knee_cadence"]
        self.right_hip_cadence = datastruct["right_hip_cadence"]
        self.right_knee_cadence = datastruct["right_knee_cadence"]
        self.period_seconds = datastruct["period_seconds"]
        self.time_offset = time.time()
	self.turn_on = False
        self.left_speed = 0
        self.right_speed =0
        self.thread = threading.Thread(target=self._daemon_loop)
        self.thread.daemon = True
        self.thread_should_quit = False
        self.thread.start()

    def set_left_speed(self, fb):  # +1 is forward, -1 is backward
        self.left_speed = np.clip(fb, -1, 1)

    def set_right_speed(self, fb):  # +1 is forward, -1 is backward
        self.right_speed = np.clip(fb, -1, 1)

#    def _daemon_loop(self):
#        while not self.thread_should_quit:
#            # note, fifty times a second seemed to freeze
#            time.sleep(0.04)  # check twenty-five times a second
#	    if self.turn_on:
#            	self.update_servo_positions()

    def update_loop(self):
	if self.turn_on:
            self.update_servo_positions()


    def update_servo_positions(self):
        now = time.time()
        t = now-self.time_offset
        phase = (t % self.period_seconds)/self.period_seconds
        for motor in self.left_knee_cadence:
            next_pos = self.get_motor_pos(phase, self.left_knee_cadence[motor])
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)
        for motor in self.right_knee_cadence:
            next_pos = self.get_motor_pos(phase, self.right_knee_cadence[motor])
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)
        for motor in self.right_hip_cadence:
            next_pos = self.right_speed*self.get_motor_pos(phase, self.right_hip_cadence[motor])
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)
        for motor in self.left_hip_cadence:
            next_pos=self.left_speed*self.get_motor_pos(phase, self.left_hip_cadence[motor])
            self.hardware[motor].setpos_fraction(next_pos)
            time.sleep(0.0005)
    def get_motor_pos(self,phase,cadence):
            return np.interp(
                phase, cadence["times"], cadence["positions"])


    def set(self, endpoint, value):
        if endpoint == "left_speed":
            self.set_left_speed(value)
            return
        if endpoint == "right_speed":
            self.set_right_speed(value)
            return
	if endpoint == "on_off":
            if value:
                self.turn_on=True
            else:
                self.turn_on=False
            return
        raise Exception("unknown endpoint {}".format(endpoint))

    def get(self, endpoint):
        if endpoint == "left_speed":
            return self.left_speed
        if endpoint == "right_speed":
            return self.right_speed
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
