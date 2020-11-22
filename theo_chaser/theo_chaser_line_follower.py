
#behavior to follow line

from theo_chaser_behaviors import DisplayCamera
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np
import logging
import random
from tape_finder import process_img_to_find_tape
import time
import torch
import torch.nn as nn
timestr = time.strftime("%Y%m%d-%H%M%S")

class Theo_Chaser_Line_Follower(DisplayCamera):
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
        self.last_action=None
        self.last_action_timestamp=time.time()
        self.start_time=time.time()
        self.spasm_mode="none"
        self.skew_act_pred_model=nn.Sequential(nn.Linear(4,4),nn.Sigmoid(),nn.Linear(4,1))
        self.skew_bm_pred_model=nn.Linear(3,2)
        self.turn_act_pred_model=nn.Sequential(nn.Linear(4,4),nn.Sigmoid(),nn.Linear(4,1))
        self.turn_bm_pred_model=nn.Linear(3,2)
        self.fb_act_pred_model=nn.Sequential(nn.Linear(4,4),nn.Sigmoid(),nn.Linear(4,1))
        self.fb_bm_pred_model=nn.Sequential(nn.Linear(3,3),nn.Sigmoid(),nn.Linear(3,2))

        modelfile=torch.load("motion_pred.pt")

        #action prediction models.  Given [ db/100, dm, m, b/100] prection [ 10*action_scale ]
        #future prediction models   Given [ b/100, m, 10*action_scale] predect next [ b/100, m]
        self.skew_act_pred_model.load_state_dict(modelfile["skew_act_pred_model"])
        self.turn_act_pred_model.load_state_dict(modelfile["turn_act_pred_model"])
        self.fb_act_pred_model.load_state_dict(modelfile["fb_act_pred_model"])
        self.skew_bm_pred_model.load_state_dict(modelfile["skew_bm_pred_model"])
        self.turn_bm_pred_model.load_state_dict(modelfile["turn_bm_pred_model"])
        self.fb_bm_pred_model.load_state_dict(modelfile["fb_bm_pred_model"])
        self.skew_act_pred_model.eval()
        self.turn_act_pred_model.eval()
        self.fb_act_pred_model.eval()
        self.skew_bm_pred_model.eval()
        self.turn_bm_pred_model.eval()
        self.fb_bm_pred_model.eval()

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
                self.spasm_mode="none"
            if key=="BTN_EAST":
                self.spasm_mode="skew"
            if key=="BTN_NORTH":
                #self.spasm_mode="turn"
                self.spasm_mode="auto"
            if key=="BTN_WEST":
                self.spasm_mode="forbac"

    def instinct_followline(self,bottom_b,m):
        time_scale=0.10
        if abs(bottom_b)>50: #too far off side to side
            b_to_translation=0.002
            side_to_side=bottom_b*b_to_translation
            print("bottom b {}".format(bottom_b))
            print("side_to_side {}".format(side_to_side))
            translation=[0,side_to_side,0,time_scale]
            scaled_translation=translation
            if side_to_side<0.4:
                scaled_translation=[0,np.sign(side_to_side)*0.4,0,time_scale*abs(side_to_side)/0.4]
        elif abs(m)>0.10:
            #Turning
            m_to_turn=0.5
            print("m {}".format(m))
            turn_power=m_to_turn*m
            print("turn power {}".format(turn_power))
            translation=[0,0,turn_power,time_scale]
            scaled_translation=translation
            if turn_power<0.4:
                scaled_translation=[0,0,np.sign(turn_power)*0.4,time_scale*abs(turn_power)/0.4]
        else: #go forward
            translation=[0.5,0,0,time_scale]
            scaled_translation=[0.5,0,0,time_scale]
        return translation,scaled_translation

    def instinct_spasm(self,bottom_b,m):
        if self.spasm_mode=="skew":
            scale=random.uniform(-1,1)
            translation=[ 0, 0.5*np.sign(scale), 0, 0.1*abs(scale) ]
            return translation

        if self.spasm_mode=="turn":
            scale=random.uniform(-1,1)
            translation=[ 0, 0, 0.5*np.sign(scale), 0.1*abs(scale) ]
            return translation

        if self.spasm_mode=="forbac":
            scale=random.uniform(-1,1)
            translation=[ 0.5*np.sign(scale), 0,0 ,0.1*abs(scale) ]
            return translation

        if self.spasm_mode=='auto':
            b=bottom_b
            print("b is {}, m is {}".format(b,m))
        #action prediction models.  Given [ db/100, dm, m, b/100] prection [ 10*action_scale ]
        #future prediction models   Given [ b/100, m, 10*action_scale] predect next [ b/100, m]
            cur_state=torch.tensor( [-b/100,-m,m,b/100]).float()
            skew_act=self.skew_act_pred_model(cur_state)
            print("desired skew {}".format(skew_act.item()/10))
            clipped_act=np.clip(skew_act.item()/10,-0.5,0.5)
            print("clipped skew {}".format(clipped_act))
            for_state=torch.tensor( [b/100,m,10*clipped_act]).float()
            new_state=self.skew_bm_pred_model(for_state)
            print("predicted new b,m {}, {}".format(b+new_state[0]*100,m+new_state[1]))

            scale=skew_act.item()
            print("scale {}".format(scale))
            translation=[ 0, 0.5*np.sign(scale),0,0.1*abs(scale)]
            return translation


            turn_act=self.turn_act_pred_model(cur_state)
            print("desired turn {}".format(turn_act.item()/10))
            clipped_act=np.clip(turn_act.item()/10,-0.5,0.5)
            for_state=torch.tensor( [b/100,m,10*turn_act.item()]).float()
            new_state=self.turn_bm_pred_model(for_state)
            print("predicted new  for turn b,m {}, {}".format(b+new_state[0]*100,m+new_state[1]))

        return None


    def act(self):
        self.handle_keys()
        ret=self.get_image()
        if ret is not None:
            fit=process_img_to_find_tape(ret)
            min_time_sample=2.0
            if fit is not None:
                b=fit.intercept
                m=fit.slope
                bottom_b=b+m*480-320
                cv.line(ret, (int(b+m*240), 240), (int(b+m*480), 480),(255,255,255), 3, cv.LINE_AA)
                #record if I had a change
                f=open("line_follow_log_{}.txt".format(timestr),"a")
                f.write("{} {} {} \n".format(self.get_image_timestamp()-self.start_time,b,m))
                f.close()
                self.last_action=None

                if self.get_image_timestamp()<self.last_action_timestamp+min_time_sample:
                #print("not ready")
                    return ret #don't act until last action could have taken
                translation=self.instinct_spasm(bottom_b,m)
                if translation==None:
                    return ret
                f=open("line_follow_action_log_{}.txt".format(timestr),"a")
                f.write("{} {} {} {} {}\n".format(time.time()-self.start_time,translation[0],translation[1],translation[2],translation[3]))
                f.close()
                self.comms.set_intention( ["drive","translate","SET"],translation)
                self.last_action_timestamp=time.time()
            return ret
        return None
        #return self.get_image()
