import Adafruit_PCA9685
import RPi.GPIO as GPIO
import logging

class GratbotServo:
    pwm=Adafruit_PCA9685.PCA9685()

    def __init__(self,datastruct):
        self.max_right=datastruct["max_right_steps"]
        self.max_left=datastruct["max_right_steps"]
        self.servo_number=datastruct["servo_number"]
        self.pwm.set_pwm_freq(60)
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
        my_steps=int(self.max_right+fraction*(self.max_left-self.max_right)/2)
        self.setpos_steps(my_steps)

class GratbotLED:
    red=[1,0,0]
    blue=[0,1,0]
    green=[0,0,1]
    pink=[1,0,1]
    cyan=[1,1,0]
    white=[1,1,0]
    yellow=[0,1,1]
    off=[0,0,0]

    def __init__(self,datastruct):
        self.red_pin=datstruct["red_pin"]
        self.green_pin=datstruct["green_pin"]
        self.blue_pin=datstruct["blue_pin"]
        GPIO.setup(self.red_pin, GPIO.OUT)
        GPIO.setup(self.green_pin, GPIO.OUT)
        GPIO.setup(self.blue_pin, GPIO.OUT)

    def set_color(self,rgb_array):
        if values[0]==0:
            GPIO.output(self.red_pin,GPIO.LOW)
        else:
            GPIO.output(self.red_pin,GPIO.HIGH)
        if values[1]==0:
            GPIO.output(self.green_pin,GPIO.LOW)
        else:
            GPIO.output(self.green_pin,GPIO.HIGH)
        if values[2]==0:
            GPIO.output(self.blue_pin,GPIO.LOW)
        else:
            GPIO.output(self.blue_pin,GPIO.HIGH)


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
        self.pwm=GPIO.PWM(self.motorpinen, 1000)

    def stop(self):
        GPIO.output(self.motorpin1,GPIO.LOW)
        GPIO.output(self.motorpin2,GPIO.LOW)
        GPIO.output(self.motorpinen,GPIO.LOW)

    def go(self,direction,speed):
        #direction is either forward or backward, speed is fraction of 1.0
        if direction==forward:
            GPIO.output(self.motorpin1, GPIO.HIGH)
            GPIO.output(self.motorpin2, GPIO.LOW)
            pwm.start(100)
            pwm.ChangeDutyCycle(speed)
        elif direction==backward:
            GPIO.output(self.motorpin1, GPIO.LOW)
            GPIO.output(self.motorpin2, GPIO.HIGH)
            pwm.start(0)
            pwm.ChangeDutyCycle(speed)
        else:
            raise Exception("invalid direction for motor")

class GratbotIRSensor:
    def __init__(self,datastruct):
        self.my_pin=datastruct["pin"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.line_pin_right,GPIO.IN)
    def get_status():
        return GPIO.input(self.my_pin)

        
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
            else:
                logging.warning("Unrecognized hardware type {}".format(datastruct[x]["type"]))
    return hardware_dat

