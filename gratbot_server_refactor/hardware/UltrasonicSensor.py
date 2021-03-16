import time
import math
import numpy as np
import threading
import RPi.GPIO as GPIO
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes

class GratbotUltrasonicSensor(GratbotSpimescape):
    def __init__(self,datastruct,hardware):
        self.trigger_pin=datastruct["trigger_pin"]
        self.echo_pin=datastruct["echo_pin"]
        self.sound_speed=datastruct["sound_speed"]
        #GPIO.setmode(GPIO.BOARD)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.echo_pin,GPIO.IN)
        self.last_avg=0
        self.last_stdev=0
        self.last_timestamp=0
        self.reading_scheduled=False
        self.update_frequency=1 #in Hz
        self.end_called=False
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()

    def _run_thread(self):
        while not self.end_called:
            total_time=1.0/self.update_frequency
            start_time=time.time()
            time_budget=0.1
            self.average_distance(time_budget)
            now=time.time()
            #wait until I should call it again
            if now-start_time<total_time:
                time.sleep(total_time-(now-start_time))

    def measure_distance(self,max_distance=4.0):
        #returns distance in meters
        timeout=0.05
        max_wait=3*max_distance/self.sound_speed
        GPIO.output(self.trigger_pin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.trigger_pin,GPIO.LOW)
        start_time=time.time()
        while not GPIO.input(self.echo_pin):
            if (time.time()-start_time)>timeout:
                break
            pass
        t1=time.time()
        start_time=time.time()
        while GPIO.input(self.echo_pin):
            if (time.time()-start_time)>timeout:
                break
            pass
        t2=time.time()
        return (t2-t1)*self.sound_speed/2

    def average_distance(self,time_budget):
        #returns average and standard deviation of n_averages distance measurements
        #print("average distance called")
        t1=time.time()
        x_sum=0
        xx_sum=0
        n_averages=0
        while (time.time()-t1)<time_budget:
            x=self.measure_distance()
            #time.sleep(0.001)
            time.sleep(0.005)
            x_sum+=x
            xx_sum+=x*x
            n_averages+=1
        avg=x_sum/n_averages
        stdev=math.sqrt(xx_sum/n_averages-avg*avg)
        self.last_avg=avg
        self.last_stdev=stdev
        self.n_averages=n_averages
        self.last_timestamp=time.time()
        return avg,stdev

    def set(self,endpoint,value):
        if endpoint=="update_frequency":
            print("setting update frequency to {}".format(float(value)))
            self.update_frequency=float(value)
        else:
            super().set(endpoint,value)

    def get_last_measurement(self):
        return { "average_distance": self.last_avg, "stdev_distance": self.last_stdev, "timestamp": self.last_timestamp, "n_averages": self.n_averages }

    def get(self,endpoint):
        time_budget=0.10
        print("get called")
        if endpoint=="last_measurement":
            "last measurement queried"
            return self.get_last_measurement()
        avg,stdev=self.average_distance(time_budget)
        #I assume the endpoint is something like "distance"
        return { "average_distance": avg, "stdev_distance": stdev }
        #avg=self.measure_distance()
        #return {"average_distance": avg}

    def get_update(self,last_time): #request changes since time as dictionary
        ret={}
        if last_time<self.last_timestamp:
            ret["last_measurement"]=self.get_last_measurement()
        return ret

    def __del__(self):
        self.end_called=true
        self.thread.join()

_all_gratbot_spimescapes["GratbotUltrasonicSensor"]=GratbotUltrasonicSensor
