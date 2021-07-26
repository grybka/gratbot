#from inputs import devices
import pygame
from Gyrus import ThreadedGyrus
import numpy as np
import threading
import logging
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class XboxControllerGyrus(ThreadedGyrus):

    def __init__(self,broker):
        pygame.init()
        pygame.joystick.init()
        self.joystick=pygame.joystick.Joystick(0)
        #self.gamepad=devices.gamepads[0]
        #self.gamepad_max_val=32768
        #self.controller_dead_zone=0.3*self.gamepad_max_val

        self.receive_thread = threading.Thread(target=self._receive_thread_loop)
        self.receive_thread.daemon = True
        super().__init__(broker)


    def start_thread_called(self):
        logger.debug("starting controller monitoring thread")
        self.receive_thread.start()

    def join_called(self):
        return self.receive_thread.join()

    def get_keys(self):
        return []

    def get_name(self):
        return "XboxControllerGyrus"

    def read_message(self,message):
        ...

    def _receive_thread_loop(self):
        logger.info("Starting controller loop")
        while not self.should_quit:
            #logger.debug("in controller loop")
            left= self.joystick.get_axis(1)
            right=self.joystick.get_axis(3)
            left_lr=self.joystick.get_axis(0)
            if abs(left)<0.2:
                left=0
            if abs(right)<0.2:
                right=0
            if abs(left_lr)<0.2:
                left_lr=0
            self.duration=0.2
            motor_command={"timestamp": time.time(),"motor_command": {"left_throttle": left,
                                                                   "right_throttle": right,
                                                                   "left_duration": self.duration,
                                                                   "right_duration": self.duration},"keys": ["motor_command"]}
            if left !=0 or right !=0:
                self.broker.publish(motor_command,"motor_command")


            servo_command={"timestamp": time.time(),"servo_command": {"servo_number":0,"delta_angle": 5*left_lr}}
            if left_lr!=0:
                self.broker.publish(servo_command,"servo_command")



            behavior_command=None
            if self.joystick.get_button(0):
                #behavior_command={"behavior_request": {"name": "trackifseen"}}
                #behavior_command={"behavior_request": {"name": "exerciseservo"}}
                ...
            if self.joystick.get_button(3):
                behavior_command={"behavior_request": {"name": "nothing"}}
            if behavior_command is not None:
                self.broker.publish(behavior_command,["behavior_request"])
            for i in range(self.joystick.get_numbuttons()):
                status=self.joystick.get_button(i)
                #if status:
                #    print("Button {} status {}".format(i,status))
            time.sleep(self.duration)
