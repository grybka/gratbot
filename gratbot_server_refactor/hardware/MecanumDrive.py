import time
import threading
import board
import numpy as np
from adafruit_motorkit import MotorKit
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes
import queue

class GratbotMecanumDrive(GratbotSpimescape):
    def __init__(self,datastruct,hardware):
        self.kit=MotorKit(i2c=board.I2C())
        self.kit._pca.frequency=1000
        self.fl_motor=self.get_kit_motor(datastruct["fl_motor"])
        self.fl_motor_sign=np.sign(datastruct["fl_motor"])
        self.fr_motor=self.get_kit_motor(datastruct["fr_motor"])
        self.fr_motor_sign=np.sign(datastruct["fr_motor"])
        self.bl_motor=self.get_kit_motor(datastruct["bl_motor"])
        self.bl_motor_sign=np.sign(datastruct["bl_motor"])
        self.br_motor=self.get_kit_motor(datastruct["br_motor"])
        self.br_motor_sign=np.sign(datastruct["br_motor"])
        self.start_time=0
        self.stop_time=0
        self.motor_active=[0,0,0]
        #self.activity_queue=queue.SimpleQueue()

        self.end_called=False
        self.motor_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()

    def _run_thread(self):
        while not self.end_called:
            time.sleep(0.005)
            if self.stop_time!=0:
                if time.time()>self.stop_time:
                    with self.motor_lock:
                        self.fl_motor.throttle=0
                        self.fr_motor.throttle=0
                        self.bl_motor.throttle=0
                        self.br_motor.throttle=0
                        #queue.put([self.start_time,time.time(),self.motor_active])
                        self.motor_active=[0,0,0]
                        self.start_time=time.time()
                        self.stop_time=0

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

    def drive_motors(self,motor_matrix):
        self.fl_motor.throttle=self.fl_motor_sign*motor_matrix[0][0]
        self.fr_motor.throttle=self.fr_motor_sign*motor_matrix[0][1]
        self.bl_motor.throttle=self.bl_motor_sign*motor_matrix[1][0]
        self.br_motor.throttle=self.br_motor_sign*motor_matrix[1][1]

    def set(self,endpoint,value):
        if endpoint=="stop":
            with self.motor_lock:
                kit.motor1.throttle=0
                kit.motor2.throttle=0
                kit.motor3.throttle=0
                kit.motor4.throttle=0
                #queue.put([self.start_time,time.time(),self.motor_active])
                self.start_time=time.time()
                self.motor_active=[0,0,0]
                self.stop_time=0
        elif endpoint=="translate":
            updown=np.array([[1,1],[1,1]])
            leftright=np.array([[1,-1],[-1,1]])
            turn_matrix=np.array([[-1,1],[-1,1]])
            sum_matrix=updown*value[0]+leftright*value[1]+turn_matrix*value[2]
             #normalize
            sum_matrix=sum_matrix/max(np.max(abs(sum_matrix)),1.0)
            with self.motor_lock:
                #queue.put([self.start_time,time.time(),self.motor_active])
                self.start_time=time.time()
                self.motor_active=[value[0],value[1],value[2]]
                if len(value)>3:
                    self.stop_time=time.time()+value[3]
                else:
                    self.stop_time=0
                self.drive_motors(sum_matrix)
        elif endpoint=="front_left":
            value=np.clip(value,-1,1)
            self.fl_motor.throttle=float(value)*self.fl_motor_sign
        elif endpoint=="front_right":
            value=np.clip(value,-1,1)
            self.fr_motor.throttle=float(value)*self.fr_motor_sign
        elif endpoint=="back_right":
            value=np.clip(value,-1,1)
            self.br_motor.throttle=float(value)*self.br_motor_sign
        elif endpoint=="back_left":
            value=np.clip(value,-1,1)
            self.bl_motor.throttle=float(value)*self.bl_motor_sign
        else:
            raise Exception("No endpoint {}".format(endpoint))

    def shut_down(self):
        self.fl_motor.throttle=0
        self.fr_motor.throttle=0
        self.bl_motor.throttle=0
        self.br_motor.throttle=0

    def get_update(self,last_time):
        ret={}
        with self.motor_lock:
            #I want to know what the motors have been doing, and how long they have been doing it since the last update

            ret["motors_active"]=self.motor_active
            #activity_list=[]
            #while not self.activity_queue.empty()==False:
            #    activity_list.append(queue.get())
            #activity_list.append([self.start_time,time.time(),self.motor_active])
            #ret["motor_activity"]=activity_list
        return ret

    def __del__(self):
        self.end_called=True
        self.thread.join()

_all_gratbot_spimescapes["GratbotMecanumDrive"]=GratbotMecanumDrive
