#motor gyrus
import threading
import logging
import board
import time
import numpy as np
from Gyrus import ThreadedGyrus
from adafruit_motorkit import MotorKit
from adafruit_motor import motor

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class MotorGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.kit=MotorKit(i2c=board.I2C())

        self.kit._pca.frequency=100
        self.thread_sleep_time=0.02
        #self.min_throttle=0.35
        self.min_throttle=0.2

        self.left_motor=self.kit.motor1
        self.right_motor=self.kit.motor2

        self.motor_lock=threading.Lock()
        self.left_motor.throttle=0
        self.right_motor.throttle=0
        self.left_run_until=0
        self.right_run_until=0
        self.left_on=True
        self.right_on=True


        self.left_motor.DECAY_MODE=motor.SLOW_DECAY #Default is fast
        self.right_motor.DECAY_MODE=motor.SLOW_DECAY #Default is fast

        self.motor_thread=None
        super().__init__(broker)

    def get_keys(self):
        return ["motor_command"]

    def get_name(self):
        return "MotorGyrus"

    def start_thread_called(self):
        self.motor_thread = threading.Thread(target=self._motor_thread_loop)
        self.motor_thread.daemon = True
        self.motor_thread.start()

    def join_called(self):
        if self.motor_thread is not None:
            return self.motor_thread.join()
        with self.motor_lock:
            self.left_motor.throttle=0
            self.right_motor.throttle=0
        return None

    def _motor_thread_loop(self):
        #this thread stops motors when they are supposed to stop
        while not self.should_quit:
            time.sleep(self.thread_sleep_time)
            now=time.time()
            #acquire most recent motor settings, set to zero if time out
            with self.motor_lock:
                if now>self.left_run_until:
                    self.left_motor_throttle=0
                if now>self.right_run_until:
                    self.right_motor_throttle=0
                left_motor_throttle=self.left_motor.throttle
                right_motor_throttle=self.right_motor.throttle
            #communicate to the motors.  This seems to take some time,
            #which is why this has its own separate thread
            self.left_motor.throttle=left_motor_throttle
            self.right_motor.throttle=right_motor_throttle
        self.left_motor.throttle=0
        self.right_motor.throttle=0


    def scale_throttle(self,throttle,duration):
        if abs(throttle)>=self.min_throttle:
            return throttle,duration
        scale=abs(throttle)/self.min_throttle
        new_duration=duration*scale
        if new_duration<self.thread_sleep_time:
            return 0,0
        return np.clip(np.sign(throttle)*self.min_throttle,-1,1),duration*scale

    def read_message(self,message):
        if "motor_command" in message:
            m=message["motor_command"]
            now=time.time()
            left_throttle,left_duration=self.scale_throttle(m["left_throttle"],m["left_duration"])
            right_throttle,right_duration=self.scale_throttle(m["right_throttle"],m["right_duration"])
            #handle low throttles with reduced duration
            with self.motor_lock:
                self.left_run_until=now+left_duration
                self.right_run_until=now+right_duration
                self.left_motor_throttle=left_throttle
                self.right_motor_throttle=right_throttle
            #self.broker.publish({"timestamp": time.time(),"motor_response": m},["motor_response"])
