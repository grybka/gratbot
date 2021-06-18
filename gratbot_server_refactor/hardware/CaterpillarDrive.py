
import time
import threading
import board
import numpy as np
from adafruit_motorkit import MotorKit
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes
import queue
import logging


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

        if "max_run_time" in datastruct:
            self.max_run_time=float(datastruct["max_run_time"])
        else:
            self.max_run_time=1 #in seconds

        self.end_called=False
        self.motor_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()

        self.left_throttle=0
        self.right_throttle=0
        self.left_duty=0
        self.right_duty=0

        self.duty_length=10
        self.duty_count=0


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
            self.duty_count+=1
            if self.duty_count>self.duty_length:
                self.duty_count=0
            with self.motor_lock:
                if self.duty_count<self.duty_length*self.left_duty:
                    self.left_motor.throttle=self.left_throttle
                else:
                    self.left_motor.throttle=0
                if self.duty_count<self.duty_length*self.right_duty:
                    self.right_motor.throttle=self.right_throttle
                else:
                    self.right_motor.throttle=0

            if self.stop_time!=0:
                if time.time()>self.stop_time:
                    print("stopping")
                    self.stop()

    def throttle_to_duty_cycle(self,throttle):
        min_throttle=0.4
        if abs(throttle)>min_throttle:
            return throttle,1
        duty=throttle/min_throttle
        return sign(throttle)*min_throttle,duty



    def set(self,endpoint,value):
        if endpoint=="stop":
            self.stop()
        elif endpoint=="go":
            if len(value)<2:
                raise Exception("Not enough arguments provided to drive motors: {}".format(value))
            with self.motor_lock:
                lt=float(np.clip(value[0],-1,1))
                rt=float(np.clip(value[1],-1,1))
                self.left_throttle,self.left_duty=self.throttle_to_duty_cycle(lt)
                self.right_throttle,self.right_duty=self.throttle_to_duty_cycle(rt)
                #print("left throttel: {} right {}".format(lt,rt))
                #self.left_motor.throttle=lt
                #self.right_motor.throttle=rt
                self.motor_active=[lt,rt]
                self.start_time=time.time()
                if len(value)>2:
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
