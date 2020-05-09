import Adafruit_PCA9685
import RPi.GPIO as GPIO
from rpi_ws281x import *
import logging
import time
import math

_all_gratbot_spimescapes={}

class GratbotSpimescape:
    def set(self,endpoint,value):
        raise Exception("set unhandled")
    def get(self,endpoint):
        raise Exception("get unhandled")
    def expose_endpoints(self,endpoint):
        #return a list of viable get and set endpoints
        return [ [],[] ]
    def update_loop(self): # called for periodic actions
        return 

class GratbotServo(GratbotSpimescape):
    pwm=None

    def __init__(self,datastruct,hardware):
	if self.pwm==None:
            self.pwm=Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(50)
        self.max_steps=datastruct["max_steps"]
        self.min_steps=datastruct["min_steps"]
        self.neutral_steps=datastruct["neutral_steps"]
        self.scale_ratio=datastruct["scale_ratio"]
        self.servo_number=datastruct["servo_number"]

    def setpos_steps(self,steps):
        steps=int(steps)
        #set the servo position by number of steps
        if steps>self.max_steps:
            steps=self.max_steps
        if steps<self.min_steps:
            steps=self.min_steps
        self.pwm.set_pwm(self.servo_number,0,steps)

    def setpos_fraction(self,fraction):
        #set the servo by fraction of full turning
        #so fraction is in [-1,1] with 0 being center
        my_steps=int(self.neutral_steps+fraction*self.scale_ratio)
        self.setpos_steps(my_steps)

    def expose_endpoints(self,endpoint):
        return [ [], ["position"] ]
    def set(self,endpoint,value):
        if endpoint=="position":
            self.setpos_fraction(value)
        if endpoint=="position_steps":
            self.setpos_steps(value)
        else:
            raise Exception("No endpoint {}".format(endpoint))
_all_gratbot_spimescapes["GratbotServo"]=GratbotServo

class GratbotLED(GratbotSpimescape):
    red=[1,0,0]
    green=[0,1,0]
    blue=[0,0,1]
    pink=[1,0,1]
    cyan=[0,1,1]
    white=[1,1,1]
    yellow=[1,1,0]
    off=[0,0,0]

    def __init__(self,datastruct):
        self.red_pin=datastruct["red_pin"]
        self.green_pin=datastruct["green_pin"]
        self.blue_pin=datastruct["blue_pin"]
        GPIO.setup(self.red_pin, GPIO.OUT)
        GPIO.setup(self.green_pin, GPIO.OUT)
        GPIO.setup(self.blue_pin, GPIO.OUT)

    def set_color(self,rgb_array):
        if rgb_array[0]==0:
            GPIO.output(self.red_pin,GPIO.HIGH)
        else:
            GPIO.output(self.red_pin,GPIO.LOW)
        if rgb_array[1]==0:
            GPIO.output(self.green_pin,GPIO.HIGH)
        else:
            GPIO.output(self.green_pin,GPIO.LOW)
        if rgb_array[2]==0:
            GPIO.output(self.blue_pin,GPIO.HIGH)
        else:
            GPIO.output(self.blue_pin,GPIO.LOW)

    def expose_endpoints(self,endpoint):
        return [ [], ["color"] ]
    def set(self,endpoint,value):
        if endpoint=="color": #TODO handle an easy string to vector conversion here
            self.set_color(value)
        else:
            raise Exception("No endpoint {}".format(endpoint))



class GratbotMotor(GratbotSpimescape):
    forward=0
    backward=1
    def __init__(self,datastruct):
        self.motorpinen=datastruct["motor_pin_en"]
        self.motorpin1=datastruct["motor_pin_1"]
        self.motorpin2=datastruct["motor_pin_2"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.motorpinen, GPIO.OUT)
        GPIO.setup(self.motorpin1, GPIO.OUT)
        GPIO.setup(self.motorpin2, GPIO.OUT)
        #pwm_frequency=1000 from adeept
        pwm_frequency=50 #this seems pretty solid
        self.pwm=GPIO.PWM(self.motorpinen, pwm_frequency)

    def stop(self):
        GPIO.output(self.motorpin1,GPIO.LOW)
        GPIO.output(self.motorpin2,GPIO.LOW)
        GPIO.output(self.motorpinen,GPIO.LOW)

    def go(self,direction,speed):
        #direction is either forward or backward, speed is fraction of 1.0
        if direction==GratbotMotor.backward:
            GPIO.output(self.motorpin1, GPIO.HIGH)
            GPIO.output(self.motorpin2, GPIO.LOW)
            self.pwm.start(100)
            self.pwm.ChangeDutyCycle(speed)
        elif direction==GratbotMotor.forward:
            GPIO.output(self.motorpin1, GPIO.LOW)
            GPIO.output(self.motorpin2, GPIO.HIGH)
            self.pwm.start(100)
            self.pwm.ChangeDutyCycle(speed)
        else:
            raise Exception("invalid direction for motor")

    def set(self,endpoint,value):
        if endpoint=="stop":
            self.stop()
        if endpoint=="speed":
            if value>=0: #forward
                if value>100.0:
                    value=100.0
                self.go(GratbotMotor.forward,value)
            else:
                value=-value
                if value>100.0:
                    value=100.0
                self.go(GratbotMotor.backward,value)
        else:
            raise Exception("No endpoint {}".format(endpoint))


class GratbotIRSensor(GratbotSpimescape):
    def __init__(self,datastruct):
        self.my_pin=datastruct["pin"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.my_pin,GPIO.IN)
    def get_status(self):
        return GPIO.input(self.my_pin)
    def get(self,endpoint):
        #I assume the endpoint is somethnig like "irsenso"
        return get_status()

class GratbotLEDStrip(GratbotSpimescape):
#this doesn't seem to work.  Why?
    def __init__(self,datastruct):
        LED_COUNT      = 12      # Number of LED pixels.
        LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
        #LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
        LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

    def set_all_color(self,color):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i,Color(color[0],color[1],color[2]))
        self.strip.show()

    def colorwipe(self,color,wait_ms=50):
        mycolor=Color(color[0],color[1],color[2])
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i,mycolor)
            self.strip.show()
            time.sleep(wait_ms/1000.0)

class GratbotUltrasonicSensor(GratbotSpimescape):
    def __init__(self,datastruct):
        self.trigger_pin=datastruct["trigger_pin"]
        self.echo_pin=datastruct["echo_pin"]
        self.sound_speed=datastruct["sound_speed"]
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trigger_pin,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.echo_pin,GPIO.IN)

    def measure_distance(self,max_distance=2.0):
        #returns distance in meters
        timeout=1.0
        max_wait=2*max_distance/self.sound_speed
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
        t1=time.time()
        x_sum=0
        xx_sum=0
        n_averages=0
        while (time.time()-t1)<time_budget:
            x=self.measure_distance()
            x_sum+=x
            xx_sum+=x*x
            n_averages+=1
        avg=x_sum/n_averages
        stdev=math.sqrt(xx_sum/n_averages-avg*avg)
        return avg,stdev

    def get(self,endpoint):
        time_budget=0.05
        avg,stdev=self.average_distance(time_budget)
        #I assume the endpoint is something like "distance"
        return { "average_distance": avg, "stdev_distance": stdev }
        #avg=self.measure_distance()
        #return {"average_distance": avg}

def create_hardware_item(datastruct):
    if "type" in datastruct:
        if datastruct["type"]=="GratbotLED":
            return GratbotLED(datastruct)
        elif datastruct["type"]=="GratbotServo":
            return GratbotServo(datastruct)
        elif datastruct["type"]=="GratbotMotor":
            return GratbotMotor(datastruct)
        elif datastruct["type"]=="GratbotIRSensor":
            return GratbotIRSensor(datastruct)
        elif datastruct["type"]=="GratbotLEDStrip":
            return GratbotLEDStrip(datastruct)
        elif datastruct["type"]=="GratbotUltrasonicSensor":
            return GratbotUltrasonicSensor(datastruct)
        else:
            logging.warning("Unrecognized hardware type {}".format(datastruct["type"]))
            return []
    else:
        logging.warning("No type for hardware {}".format(datastruct["type"]))
        return []


def create_hardware(datastruct):
    hardware_dat={}
    for x in datastruct.keys():
        logging.info("hardware creating {}".format(x))
        if datastruct[x]["type"] in _all_gratbot_spimescapes:
            hardware_dat[x]=_all_gratbot_spimescapes[datastruct[x]["type"]](datastruct[x],hardware_dat)
	    hardware_dat[x].type=datastruct[x]["type"]
        else:
            logging.warning("Unrecognized hardware {}".format(x))
        #hardware_dat[x]=create_hardware_item(datastruct[x])
    return hardware_dat

class GratbotHardwareThread():
    def __init__(self,hardware):
        self.hardware=hardware
        self.thread=threading.Thread(target=self._daemon_loop)
        self.thread.daemon = True
        self.thread_should_quit = False
        self.thread.start()
    def _daemon_loop(self):
        while not self.thread_should_quit:
            time.sleep(0.04)
            for key in hardware:
                hardware[key].update_loop()

_gratbot_hardware_thread = 0
def start_hardware_thread(hardware):
    _gratbot_hardware_thread = threading.Thread(target=_gratbot_hardware_loop)
