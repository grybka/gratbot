from theo_chaser_behaviors import DisplayCamera
from theo_chaser_object_tagger import Theo_Chaser_Object_Tagger
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np
import logging
from tape_finder import process_img_to_find_tape
from tape_finder import process_img_to_highlight_tape

class XBoxControl(DisplayCamera):
    def __init__(self,comms):
        super().__init__(comms)
        self.gamepad=devices.gamepads[0]
        self.new_values={}
        self.values={}
        self.ok_keys=["ABS_X","ABS_Y","ABS_RX","ABS_RY"]
        self.control_mode="legs" #or camera
        for key in self.ok_keys:
            self.values[key]=0
            self.new_values[key]=0
        self.keys=[]
        self.gamepad_max_val=32768
        self.thread=threading.Thread(target=self._daemon_loop)
        self.thread_daemon = True
        self.thread_should_quit=False
        self.values_lock=threading.Lock()
        self.thread.start()
        self.max_speed=80
        self.controller_dead_zone=0.2*self.gamepad_max_val
        self.tagger=Theo_Chaser_Object_Tagger()
        #self.image_mode="uvl" #as opposed to image_tagger
        #self.image_mode="tape" #as opposed to image_tagger
        self.image_mode="image_tagger" #as opposed to image_tagger


    def _daemon_loop(self):
        while not self.thread_should_quit:
            try:
                events = self.gamepad.read()
            except EOFError:
                print("gamepad failure ")
                events = []
            self.values_lock.acquire()
            for event in events:
                if event.ev_type=="Absolute":
                    if event.code in self.ok_keys:
                        x=0
                        if abs(event.state)<self.controller_dead_zone:
                            x=0
                        elif event.state>0:
                            x=round((event.state-self.controller_dead_zone)/((self.gamepad_max_val-self.controller_dead_zone)),1)
                        else:
                            x=round((event.state+self.controller_dead_zone)/((self.gamepad_max_val-self.controller_dead_zone)),1)
                        self.new_values[event.code]=x
                        #print("{} set to {}".format(event.code,self.new_values[event.code]))
                        #print("old value {} {}".format(event.code,self.values[event.code]))
                if event.ev_type=="Key" and event.state==0:
                    self.keys.append(event.code)
            self.values_lock.release()
            time.sleep(0.01)

    def shut_down(self):
        self.thread_should_quit=True
        self.thread.join()

    def handle_keys(self):
        self.values_lock.acquire()
        changed=False
        for key in self.ok_keys:
            if self.new_values[key]!=self.values[key]:
                changed=True
                self.values[key]=self.new_values[key]
        pressed_keys=np.unique(self.keys)
        self.keys=[]
        self.values_lock.release()
        if changed:
            translation=[ self.values["ABS_RY"] , self.values["ABS_RX"], -self.values["ABS_X"] ]
            logging.info("Sending [{},{},{}]".format(translation[0],translation[1],translation[2]))
            self.comms.set_intention( ["drive","translate","SET"],translation)
        for key in pressed_keys:
            if key=="BTN_SOUTH":
                print("changing mode")
                print("was {} now ...".format(self.image_mode))
                if self.image_mode=='uvl':
                    print("tape")
                    self.image_mode='tape'
                elif self.image_mode=='tape':
                    print("uvl")
                    self.image_mode='uvl'
        #    if key=="BTN_NORTH":
        #    if key=="BTN_EAST":

    def act(self):
        self.handle_keys()
        ret=self.get_image()
        if ret is not None:
            if self.image_mode=='image_tagger':
                return self.tagger.draw_bboxes(ret)
            if self.image_mode=='uvl':
                converted=cv.cvtColor(ret, cv.COLOR_BGR2HSV)
                xtarget=320
                ytarget=240+120
                text="({})".format(converted[ytarget][xtarget])
                converted=cv.line(converted,(xtarget-10,ytarget),(xtarget+10,ytarget),(255,255,255),1)
                converted=cv.line(converted,(xtarget,ytarget-10),(xtarget,ytarget+10),(255,255,255),1)
                # font
                font = cv.FONT_HERSHEY_SIMPLEX
                # fontScale
                fontScale = 1
                color = (255, 255, 255) # (10,50,110)
                thickness = 2
                converted=cv.putText(converted,text,(320,240),font,fontScale,color,thickness,cv.LINE_AA)
                return converted
            if self.image_mode=='tape':
                #fit=process_img_to_find_tape(ret)
                fit=process_img_to_highlight_tape(ret)
                if fit is not None:
                    b=fit.intercept
                    m=fit.slope
                    cv.line(ret, (int(b+m*240), 240), (int(b+m*480), 480),(255,255,255), 3, cv.LINE_AA)
                return ret
        return None
        #return self.get_image()
