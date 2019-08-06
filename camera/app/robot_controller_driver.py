import numpy as np
import cv2 as cv
import logging
import time
import threading
from xbox360controller import Xbox360Controller

from GratbotClient import GratbotClient
from GratbotVideoConnectionUV4L import GratbotVideoConnectionUV4L
from GratbotObjectFinder import GratbotObjectFinder

root = logging.getLogger()
root.setLevel(logging.INFO)

camera_yaw_intent=0 #
camera_yaw_intent_lock=threading.Lock()
camera_pitch_intent=0
camera_pitch_intent_lock=threading.Lock()

class GratbotComms:
    def __init__(self):
        self.client=GratbotClient("10.0.0.5",9999)
        self.client.connect()
        self._camera_pitch=420
        self._camera_yaw=370
        self.camera_yaw_intent=0 #
        self.camera_yaw_intent_lock=threading.Lock()
        self.camera_pitch_intent=0
        self.camera_pitch_intent_lock=threading.Lock()
        self.wheel_yaw_neutral=370
        self.old_wheel_yaw=self.wheel_yaw_neutral
        self.wheel_yaw_spread=130
        self.wheel_yaw=0 #
        self.wheel_yaw_lock=threading.Lock()
        self.motor_speed=0 #
        self.motor_speed_lock=threading.Lock()

        self.blink_l_led=False
        self.blink_r_led=False
        self.l_led_on=True
        self.r_led_on=True

        self.should_quit=False
        self.thread=threading.Thread(target=self._loop)
        self.thread.daemon=True
        self.thread.start()

    def set_camera_yaw_intent(self,value):
        self.camera_yaw_intent_lock.acquire()
        self.camera_yaw_intent=value
        self.camera_yaw_intent_lock.release()
    
    def get_camera_yaw_intent(self):
        self.camera_yaw_intent_lock.acquire()
        ret=self.camera_yaw_intent
        self.camera_yaw_intent_lock.release()
        return ret

    def set_camera_pitch_intent(self,value):
        self.camera_pitch_intent_lock.acquire()
        self.camera_pitch_intent=value
        self.camera_pitch_intent_lock.release()

    def get_camera_pitch_intent(self):
        self.camera_pitch_intent_lock.acquire()
        ret=self.camera_pitch_intent
        self.camera_pitch_intent_lock.release()
        return ret

    def set_wheel_yaw(self,value):
        self.wheel_yaw_lock.acquire()
        self.wheel_yaw=value
        self.wheel_yaw_lock.release()
    
    def get_wheel_yaw(self):
        self.wheel_yaw_lock.acquire()
        ret=self.wheel_yaw
        self.wheel_yaw_lock.release()
        return ret

    def set_motor_speed(self,value):
        self.motor_speed_lock.acquire()
        self.motor_speed=value
        self.motor_speed_lock.release()
    
    def get_motor_speed(self):
        self.motor_speed_lock.acquire()
        ret=self.motor_speed
        self.motor_speed_lock.release()
        return ret


    def _loop(self):
        while not self.should_quit:
            time.sleep(0.1) #only try to message at 10 Hz
            camera_step_size=5
            old_yaw=self._camera_yaw
            old_pitch=self._camera_pitch
            self._camera_yaw+=camera_step_size*self.get_camera_pitch_intent()
            self._camera_pitch+=camera_step_size*self.get_camera_yaw_intent()
            if int(self._camera_pitch)!=int(old_pitch):
                self.client.send_message(["camera_pitch_servo","position_steps"],"SET",int(self._camera_pitch))
                logging.info("pitching {}".format(self._camera_pitch))
            if int(self._camera_yaw)!=int(old_yaw):
                self.client.send_message(["camera_yaw_servo","position_steps"],"SET",int(self._camera_yaw))
                logging.info("yawing {}".format(self._camera_yaw))
            new_wheel_yaw=self.get_wheel_yaw()*self.wheel_yaw_spread+self.wheel_yaw_neutral
            if int(new_wheel_yaw)!=int(self.old_wheel_yaw):
                self.client.send_message(["wheel_turn_servo","position_steps"],"SET",int(new_wheel_yaw))
                self.old_wheel_yaw=new_wheel_yaw
                logging.info("wheel yawing {}".format(self.old_wheel_yaw))
            self.client.send_message(["wheel_motor","speed"],"SET",int(100*self.get_motor_speed()))
            #if self.get_motor_speed()!=0:
                #logging.info("motor speed set to {}".format(self.get_motor_speed()))
            if self.blink_l_led==True:
                logging.info("swapping l led")
                self.blink_l_led=False
                if self.l_led_on:
                    self.l_led_on=False
                    self.client.send_message(["left_front_led","color"],"SET",[0,0,0])
                else:
                    self.l_led_on=True
                    self.client.send_message(["left_front_led","color"],"SET",[1,1,1])
            if self.blink_r_led==True:
                logging.info("swapping l led")
                self.blink_r_led=False
                if self.r_led_on:
                    self.r_led_on=False
                    self.client.send_message(["right_front_led","color"],"SET",[0,0,0])
                else:
                    self.r_led_on=True
                    self.client.send_message(["right_front_led","color"],"SET",[1,1,1])
 
                

    def stop(self):
        self.should_quit=True
        if self.thread.is_alive():
            self.thread.join()
        self.client.disconnect()

    def __del__(self):
        self.client.disconnect()

#connect to bot controls
gratbot_comms=GratbotComms()
#gratbot=GratbotClient("10.0.0.5",9999)
#gratbot.connect()

#connect to camera
cv.namedWindow("preview")
video=GratbotVideoConnectionUV4L("http://10.0.0.5:8080/stream/video.mjpeg")
frame_width=video.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frame_height=video.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width,frame_height))



#define controller callbacks
def on_axis_l_moved(axis):
    #axis_name can be axis_r or axis_l
    #if axis.name=="axis_l": #lets make this move the camera
    vx=axis.x
    vy=axis.y
    if abs(vx)<0.5:
        vx=0
    if abs(vy)<0.5:
        vy=0
    gratbot_comms.set_camera_yaw_intent(-vx)
    gratbot_comms.set_camera_pitch_intent(-vy)

def on_axis_r_moved(axis):
    vx=axis.x
    vy=axis.y
    if abs(vx)<0.5:
        vx=0
    if abs(vy)<0.5:
        vy=0
    gratbot_comms.set_wheel_yaw(-vx)
    gratbot_comms.set_motor_speed(-vy)

left_led_on=True
right_led_on=True

def on_button_a_pressed(button):
    gratbot_comms.blink_l_led=True

def on_button_b_pressed(button):
    gratbot_comms.blink_r_led=True



#connect to controller
controller=Xbox360Controller(0, axis_threshold=0.2)
controller.axis_l.when_moved=on_axis_l_moved
controller.axis_r.when_moved=on_axis_r_moved
controller.button_a.when_pressed = on_button_a_pressed
controller.button_b.when_pressed = on_button_b_pressed

#start the image finder 
imagefinder=GratbotObjectFinder(video)
cv2.namedWindow("object_finder")

#actuator loop here
try:
    cycle_counter=0
    while True:
        cycle_counter+=1
        myframe=video.read()
        myframe=cv.resize(myframe,None,fx=4,fy=4)
        cv.imshow("preview",myframe)
        objframe=imagefinder.get_processed_frame()
        objframe=cv.resize(objframe,None,fx=4,fy=4)
        cv.imshow("object_finder",objframe)
        key=cv.waitKey(10)
        #send commands (could be different thread)
        
except KeyboardInterrupt:
    logging.warning("Keyboard Exception Program Ended, exiting")
finally:        
    video.stop()
    controller.close()
    gratbot_comms.stop()
