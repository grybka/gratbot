
import time
import threading
import board
import numpy as np
from adafruit_motorkit import MotorKit
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes
import queue


class GratbotCaterpillarDrive(GratbotSpimescape):
    def __init__(self,datastruct,hardware):
        self.kit=MotorKit(i2c=board.I2C())
        self.kit._pca.frequency=1000
        self.left_motor=self.get_kit_motor(datastruct["left_motor"])
        self.left_motor_sign=np.sign(datastruct["left_motor"])
        self.right_motor=self.get_kit_motor(datastruct["right_motor"])
        self.right_motor_sign=np.sign(datastruct["right_motor"])

        self.start_time=0
        self.stop_time=0
        self.motor_active=[0,0]

        self.max_run_time=1 #in seconds

        self.end_called=False
        self.motor_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()


    def get_kit_motor(self,integer):
        if abs(integer)==1:
            return self.kit.motor1
        if abs(integer)==2:
            return self.kit.motor2
        if abs(integer)==3:
            return self.kit.motor3
        if abs(integer)==4:
            return self.kit.motor4
        raise Exception("Unknown motor {}".format(integer))

    def stop(self):
        with self.motor_lock:
            self.left_motor.throttle=0
            self.right_motor.throttle=0
            self.motor_active=[0,0]
            self.start_time=time.time()
            self.stop_time=0

    def _run_thread(self):
        while not self.end_called:
            time.sleep(0.005)
            if self.stop_time!=0:
                if time.time()>self.stop_time:
                    self.stop()

    def set(self,endpoint,value):
        if endpoint=="stop":
            self.stop()
        elif endpoint=="go":
            if len(value)<2:
                raise Exception("Not enough arguments provided to drive motors: {}".format(value))
            with self.motor_lock:
                self.left_motor.throttle=np.clip(value[0],-1,1)
                self.right_motor.throttle=np.clip(value[1],-1,1)
                self.motor_active=[np.clip(value[0],-1,1),np.clip(value[1],-1,1)]
                self.start_time=time.time()
                if len(value>2):
                    self.stop_time=self.start_time+min(value[2],self.max_run_time)
                else:
                    self.stop_time=self.start_time+self.max_run_time
        else:
            raise Exception("No endpoint {}".format(endpoint))

    def get_update(self,last_time):
        ret={}
        with self.motor_lock:
            #I want to know what the motors have been doing, and how long they have been doing it since the last update
            nowtime=time.time()
            duration=nowtime-self.start_time
            ret["motors_active"]=[self.motor_active[0],self.motor_active[1],duration]
        return ret

    def __del__(self):
        self.end_called=True
        self.thread.join()

_all_gratbot_spimescapes["GratbotCaterpillarDrive"]=GratbotCaterpillarDrive
