import Adafruit_PCA9685
import RPi.GPIO as GPIO
from rpi_ws281x import *
import logging

class GratbotServo:
    pwm=Adafruit_PCA9685.PCA9685()

    def __init__(self,datastruct):
        self.max_right=datastruct["max_right_steps"]
        self.max_left=datastruct["max_left_steps"]
        self.servo_number=datastruct["servo_number"]
        self.pwm.set_pwm_freq(60)
        #print("servo  {} max_left {}".format(self.servo_number,self.max_left))
        #print("servo {} max_right {}".format(self.servo_number,self.max_right))
#self.max_right=280
#self.middle=370
#self.max_left=500
#self.servo_number=1

    def setpos_steps(self,steps):
        #set the servo position by number of steps
        if steps>self.max_left:
            steps=self.max_left
        if steps<self.max_right:
            steps=self.max_right
        self.pwm.set_pwm(self.servo_number,0,steps)

    def setpos_fraction(self,fraction):
        #set the servo by fraction of full turning
        #so fraction is in [-1,1] with 0 being center
        center=0.5*(self.max_left+self.max_right)
        halfspan=0.5*abs(self.max_left-self.max_right)
        my_steps=int(center+fraction*halfspan)
        self.setpos_steps(my_steps)

class GratbotLED:
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


class GratbotMotor:
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

class GratbotIRSensor:
    def __init__(self,datastruct):
        self.my_pin=datastruct["pin"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.my_pin,GPIO.IN)
    def get_status(self):
        return GPIO.input(self.my_pin)

class GratbotLEDStrip:
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

def create_hardware(datastruct):
    hardware_dat={}
    for x in datastruct.keys():
        if "type" in datastruct[x]:
            logging.info("hardware creating {}".format(x))
            if datastruct[x]["type"]=="GratbotLED":
                hardware_dat[x]=GratbotLED(datastruct[x])
            elif datastruct[x]["type"]=="GratbotServo":
                hardware_dat[x]=GratbotServo(datastruct[x])
            elif datastruct[x]["type"]=="GratbotMotor":
                hardware_dat[x]=GratbotMotor(datastruct[x])
            elif datastruct[x]["type"]=="GratbotIRSensor":
                hardware_dat[x]=GratbotIRSensor(datastruct[x])
            elif datastruct[x]["type"]=="GratbotLEDStrip":
                hardware_dat[x]=GratbotLEDStrip(datastruct[x])
            else:
                logging.warning("Unrecognized hardware type {}".format(datastruct[x]["type"]))
    return hardware_dat

