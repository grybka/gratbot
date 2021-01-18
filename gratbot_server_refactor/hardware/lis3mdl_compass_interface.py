import time
import board
import busio
import adafruit_lis3mdl
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes

class GratbotMagnetometer(GratbotSpimescape):
    def __init__(self, datastruct, hardware):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lis3mdl.LIS3MDL(i2c)
        self.sensor.range=adafruit_lis3mdl.Range.RANGE_4_GAUSS
        #self.sensor.range=adafruit_lis3mdl.Range.RANGE_8_GAUSS
        #self.sensor.range=adafruit_lis3mdl.Range.RANGE_12_GAUSS

    def set(self, endpoint, value):
      return None

    def get(self, endpoint):
        if endpoint=="field":
            return self.sensor.magnetic
        raise Exception("unknown camera endpoint {}".format(endpoint))

    def get_update(self,last_time): #request changes since time as dictionary
        ret={}
        ret["b_field"]=self.sensor.magnetic
        return ret

_all_gratbot_spimescapes["Magnetometer"]=GratbotMagnetometer
