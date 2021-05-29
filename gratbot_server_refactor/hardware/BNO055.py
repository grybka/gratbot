import time
import board
import busio
import adafruit_bno055

from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes

class GratbotPositionSensor(GratbotSpimescape):
    def __init__(self, datastruct, hardware):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.end_called=False

        #self.data_lock=threading.Lock()
        #self.thread = threading.Thread(target=self._run_thread)
        #self.thread.daemon = True
        #self.thread.start()

        #self.accel_list=[]
        #self.gyro_list=[]
        #self.magnetic_list=[]

    def set(self, endpoint, value):
        return None

    def get_update(self,last_time): #request changes since time as dictionary
        ret={}
        ret["b_field"]=self.sensor.magnetic
        ret["acceleration"]=self.sensor.acceleration
        ret["gyro"]=self.sensor.gyro
        ret["calibration"]=self.sensor.calibration_status
        return ret

_all_gratbot_spimescapes["PositionSensor"]=GratbotPositionSensor
