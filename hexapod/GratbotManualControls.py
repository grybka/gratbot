from GratbotBehaviors import GratbotBehavior
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np

class XBoxControl(GratbotBehavior):
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
                        self.new_values[event.code]=2*round(event.state/(2*self.gamepad_max_val),1)
                        #print("{} set to {}".format(event.code,self.new_values[event.code]))
                        #print("old value {} {}".format(event.code,self.values[event.code]))
                if event.ev_type=="Key" and event.state==0:
                    self.keys.append(event.code)
            self.values_lock.release()
            time.sleep(0.01)

    def shut_down(self):
        self.thread_should_quit=True
        self.thread.join()


    def act(self, video_frame):
        now=time.time()
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
            if self.control_mode=="legs":
                if self.values["ABS_Y"]==0 and self.values["ABS_RY"]==0:
                    self.comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
                else:
                    self.comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                    self.comms.set_intention( [ "leg_controller","left_speed", "SET" ],self.values["ABS_Y"])
                    self.comms.set_intention( [ "leg_controller","right_speed", "SET" ], self.values["ABS_RY"])
            elif self.control_mode=="camera":
                if self.values["ABS_RY"]!=0:
                    self.comms.set_intention( ["camera_y","position_delta","SET" ], self.values["ABS_RY"]*10)
                if self.values["ABS_RX"]!=0:
                    self.comms.set_intention( ["camera_x","position_delta","SET" ], self.values["ABS_RX"]*10)
        for key in pressed_keys:
            if key=="BTN_SOUTH":
                cv.imwrite(time.strftime("%Y%m%d-%H%M%S.jpg"),video_frame)
                print("saved!")
            if key=="BTN_NORTH":
                print("camera mode")
                self.control_mode="camera"
            if key=="BTN_EAST":
                print("leg mode")
                self.control_mode="legs"

        return video_frame
