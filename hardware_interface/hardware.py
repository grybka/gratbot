import Adafruit_PCA9685
import RPi.GPIO as GPIO
from rpi_ws281x import *
import logging
import time
import math
import threading
import numpy
import board
from adafruit_motorkit import MotorKit
import numpy as np

_all_gratbot_spimescapes={}

class GratbotSpimescape:
    def set(self,endpoint,value):
        raise Exception("set unhandled")
    def get(self,endpoint):
        raise Exception("get unhandled")
    def expose_endpoints(self,endpoint):
        #return a list of viable get and set endpoints
        return [ [],[] ]
    def shut_down(self):
        return
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
        self.last_steps = datastruct["neutral_steps"]
        if "max_steps_per_command" in datastruct:
            self.max_step_per_command = datastruct["max_steps_per_command"]
        else:
            self.max_step_per_command = 20

    def setpos_steps(self,steps):
        steps=int(steps)
        #set the servo position by number of steps
        steps=numpy.clip(steps,max(self.last_steps-self.max_step_per_command,self.min_steps),min(self.last_steps+self.max_step_per_command,self.max_steps))
        #if steps>self.max_steps:
        #    steps=self.max_steps
        #if steps<self.min_steps:
        #    steps=self.min_steps
        self.pwm.set_pwm(self.servo_number,0,steps)
        self.last_steps=steps

    def setpos_fraction(self,fraction):
        #set the servo by fraction of full turning
        #so fraction is in [-1,1] with 0 being center
        my_steps=int(self.neutral_steps+fraction*self.scale_ratio)
        self.setpos_steps(my_steps)

    def expose_endpoints(self,endpoint):
        return [ [], ["position"] ]
    def get(self, endpoint):
        if endpoint=="position_steps":
            return self.last_steps
        raise Exception("No endpoint {}".format(endpoint))
    def set(self,endpoint,value):
        if endpoint=="position_delta":
            self.setpos_steps(value+self.last_steps)
            return
        if endpoint=="position":
            self.setpos_fraction(value)
            return
        if endpoint=="position_steps":
            self.setpos_steps(value)
            return
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
    def __init__(self,datastruct,hardware):
        self.motorpinen=datastruct["motor_pin_en"]
        self.motorpin1=datastruct["motor_pin_1"]
        self.motorpin2=datastruct["motor_pin_2"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.motorpinen, GPIO.OUT)
        GPIO.setup(self.motorpin1, GPIO.OUT)
        GPIO.setup(self.motorpin2, GPIO.OUT)
        pwm_frequency=1000 #from adeept
        #pwm_frequency=50 #this seems pretty solid
        pwm_frequency=30
        self.pwm=GPIO.PWM(self.motorpinen, pwm_frequency)

    def stop(self):
        self.go(GratbotMotor.forward,0)
        GPIO.output(self.motorpin1,GPIO.LOW)
        GPIO.output(self.motorpin2,GPIO.LOW)
        GPIO.output(self.motorpinen,GPIO.LOW)

    def go(self,direction,speed):
        #direction is either forward or backward, speed is fraction of 1.0
        if direction==GratbotMotor.backward:
            GPIO.output(self.motorpin1, GPIO.HIGH)
            GPIO.output(self.motorpin2, GPIO.LOW)
            self.pwm.start(100)
            time.sleep(0.05)
            self.pwm.ChangeDutyCycle(speed)
        elif direction==GratbotMotor.forward:
            GPIO.output(self.motorpin1, GPIO.LOW)
            GPIO.output(self.motorpin2, GPIO.HIGH)
            self.pwm.start(100)
            time.sleep(0.05)
            self.pwm.ChangeDutyCycle(speed)
        else:
            raise Exception("invalid direction for motor")

    def set(self,endpoint,value):
        if endpoint=="stop":
            self.stop()
        elif endpoint=="speed":
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
_all_gratbot_spimescapes["GratbotMotor"]=GratbotMotor

class GratbotMecanumDrive(GratbotSpimescape):
    def __init__(self,datastruct,hardware):
        self.kit=MotorKit(i2c=board.I2C())
        self.kit._pca.frequency=100
        self.fl_motor=self.get_kit_motor(datastruct["fl_motor"])
        self.fl_motor_sign=np.sign(datastruct["fl_motor"])
        self.fr_motor=self.get_kit_motor(datastruct["fr_motor"])
        self.fr_motor_sign=np.sign(datastruct["fr_motor"])
        self.bl_motor=self.get_kit_motor(datastruct["bl_motor"])
        self.bl_motor_sign=np.sign(datastruct["bl_motor"])
        self.br_motor=self.get_kit_motor(datastruct["br_motor"])
        self.br_motor_sign=np.sign(datastruct["br_motor"])
        self.stop_time=0

    def update_loop(self):
        if self.stop_time!=0:
            if time.time()>self.stop_time:
                self.fl_motor.throttle=0
                self.fr_motor.throttle=0
                self.bl_motor.throttle=0
                self.br_motor.throttle=0
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
            kit.motor1.throttle=0
            kit.motor2.throttle=0
            kit.motor3.throttle=0
            kit.motor4.throttle=0
        elif endpoint=="translate":
            updown=np.array([[1,1],[1,1]])
            leftright=np.array([[1,-1],[-1,1]])
            turn_matrix=np.array([[-1,1],[-1,1]])
            sum_matrix=updown*value[0]+leftright*value[1]+turn_matrix*value[2]
            if len(value)>3:
                self.stop_time=time.time()+value[3]
            else:
                self.stop_time=0

            #normalize
            sum_matrix=sum_matrix/max(np.max(abs(sum_matrix)),1.0)
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


_all_gratbot_spimescapes["GratbotMecanumDrive"]=GratbotMecanumDrive

class GratbotIRSensor(GratbotSpimescape):
    def __init__(self,datastruct):
        self.my_pin=datastruct["pin"]
        GPIO.setwarnings(False)
        #GPIO.setmode(GPIO.BOARD)
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
        #GPIO.setmode(GPIO.BOARD)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.echo_pin,GPIO.IN)
        self.last_avg=0
        self.last_stdev=0
        self.last_timestamp=0
        self.reading_scheduled=False

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
            time.sleep(0.001)
            x_sum+=x
            xx_sum+=x*x
            n_averages+=1
        avg=x_sum/n_averages
        stdev=math.sqrt(xx_sum/n_averages-avg*avg)
        self.last_avg=avg
        self.last_stdev=stdev
        self.last_timestamp=time.time()
        return avg,stdev


    def update_loop(self): # called for periodic actions
        if self.reading_scheduled:
            time_budget=0.05
            self.average_distance(time_budget)
            self.reading_scheduled=False
        return

    def set(self,endpoint,value):
        #for now, anything just tells it to take a reading

    def get(self,endpoint):
        if endpoint="last_measurement":
            return { "average_distance": self.last_avg, "stdev_distance": self.last_stdev, "timestamp": last_time }
        time_budget=0.05
        avg,stdev=self.average_distance(time_budget)
        #I assume the endpoint is something like "distance"
        return { "average_distance": avg, "stdev_distance": stdev }
        #avg=self.measure_distance()
        #return {"average_distance": avg}


_all_gratbot_spimescapes["GratbotUltrasonicSensor"]=GratbotUltrasonicSensor

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
            for key in self.hardware:
                self.hardware[key].update_loop()

_gratbot_hardware_thread = 0
def start_hardware_thread(hardware):
    _gratbot_hardware_thread = threading.Thread(target=_gratbot_hardware_loop)
