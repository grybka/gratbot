
import time
import board
import busio
import adafruit_bno055
import threading
from collections import deque
import numpy as np

from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes

class Gratbot9DOFSensor(GratbotSpimescape):
    def __init__(self, datastruct, hardware):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.end_called=False

        self.data_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()
        self.deque_size=10

        #self.accel_deque=[ deque([],self.deque_size),deque([],self.deque_size),deque([],self.deque_size)]
        self.accel_deque=deque( [ [0,0,0] ],self.deque_size)
        self.gyro_deque=deque( [ [0,0,0] ],self.deque_size)
        self.mag_deque=deque( [ [0,0,0] ],self.deque_size)

        #self.accel_list=[]
        #self.gyro_list=[]
        #self.magnetic_list=[]

    def _run_thread(self):
        while not self.end_called:
            time.sleep(0.01)
            with self.data_lock:
                mag=self.sensor.magnetic
                accel=self.sensor.linear_acceleration
                gyro=self.sensor.gyro
                if None not in mag:
                    self.mag_deque.appendleft(mag)
                if None not in accel:
                    self.accel_deque.appendleft(accel)
                if None not in gyro:
                    self.gyro_deque.appendleft(gyro)

    def __del__(self):
        self.end_called=True
        self.thread.join()

    def set(self, endpoint, value):
        return None

    def get_update(self,last_time): #request changes since time as dictionary
        ret={}
        with self.data_lock:
            ret["b_field"]=(np.sum(self.mag_deque,axis=0)/self.deque_size).tolist()
            ret["b_field_stdev"]=np.std(self.mag_deque,axis=0).tolist()
            ret["b_field_inst"]=self.mag_deque[0]
            #print("accel deque {}".format(self.accel_deque))
            ret["acceleration"]=(np.sum(self.accel_deque,axis=0)/self.deque_size).tolist()
            ret["acceleration_stdev"]=np.std(self.accel_deque,axis=0).tolist()
            ret["acceleration_inst"]=self.accel_deque[0]
            ret["gyro"]=(np.sum(self.gyro_deque,axis=0)/self.deque_size).tolist()
            ret["gyro_stdev"]=np.std(self.gyro_deque,axis=0).tolist()
            ret["gyro_inst"]=self.gyro_deque[0]
            ret["calibration"]=self.sensor.calibration_status
        return ret

_all_gratbot_spimescapes["9DOFSensor"]=Gratbot9DOFSensor
