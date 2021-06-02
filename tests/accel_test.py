
import time
import board
import busio
import adafruit_bno055


i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)


time_interval=0.05
last_time=time.time()

with open("accelfile.txt",'w') as datfile:
    while(True):
        t=time.time()
        if t>last_time+time_interval:
            outstr="{} ".format(t)
            outstr=outstr+"{} {} {} ".format(sensor.magnetic[0],sensor.magnetic[1],sensor.magnetic[2])
            outstr=outstr+"{} {} {} ".format(sensor.acceleration[0],sensor.acceleration[1],sensor.acceleration[2])
            outstr=outstr+"{} {} {} ".format(sensor.gyro[0],sensor.gyro[1],sensor.gyro[2])
            datfile.write(outstr+"\n")
