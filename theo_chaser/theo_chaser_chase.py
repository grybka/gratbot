#behavior to follow person

from theo_chaser_behaviors import DisplayCamera
from theo_chaser_behaviors import DisplayCameraUV4L
from theo_chaser_object_tagger import Theo_Chaser_Object_Tagger
import time
from inputs import devices
import threading
import cv2 as cv
import numpy as np
import logging

class Theo_Chaser_Chase(DisplayCamera):
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

    def follow_objects(self,video_objects):
        ret_objects=[]
        for x in video_objects:
            if x["label"] != "person" or x["confidence"]<0.5:
                continue
            pos,wh=self.tagger.get_obj_loc_width(x)
            target_width=0.6
            pixel_to_turn=-0.8
            #TODO this has trouble follwing me because it hits the bottom of the usable speed

            width_to_fb=3.0
            print("chasing {} at {} wh {}".format(x["label"],pos,wh))
            time_scale=0.1
            translation=[ width_to_fb*(target_width-wh[0]), 0, pixel_to_turn*pos[0],time_scale ]
            if np.max(np.abs(translation))<0.8:
                my_scale=0.8/np.max(np.abs(translation))
                translation[0]=translation[0]*my_scale
                translation[1]=translation[1]*my_scale
                translation[2]=translation[2]*my_scale
                time_scale=time_scale/my_scale
                translation[3]=time_scale

            #translation[0]=0 #no fb
            print("translaation {}".format(translation))
            self.comms.set_intention( ["drive","translate","SET"],translation)
            ret_objects.append(x)

            break
        return ret_objects

    def act(self):
        self.handle_keys()
        ret=self.get_image()
        if ret is not None:
            video_objects=self.tagger.tag_objects(ret)
            self.follow_objects(video_objects)
            return self.tagger.draw_bboxes(ret,video_objects)
        return None
        #return self.get_image()


class Theo_Chaser_Chase_MK2(DisplayCameraUV4L):
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
        self.last_last_image_timestamp=0
        #self.comms.set_intention(["ultrasonic_sensor","update_frequency","SET"],1)
        self.comms.set(["ultrasonic_sensor","update_frequency"],1)
        #for logging data
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        self.start_time=time.time()
        self.object_log_fname="logs/learn_and_track_object_log_{}.txt".format(self.timestr)


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
            #self.comms.set_intention( ["drive","translate","SET"],translation)
            self.comms.set( ["drive","translate"],translation)

    def interpret_translation(self,translation):
        time_base=0.1
        power_cut=0.6
        min_cut=0.05
        if(np.max(np.abs(translation))<min_cut):
            return [0,0,0,time_base]
        response=[translation[0],translation[1],translation[2],time_base]
        if(np.max(np.abs(translation))<power_cut):
            scaling=power_cut/np.max(np.abs(translation))
            response=[translation[0]*scaling,translation[1]*scaling,translation[2]*scaling,time_base/scaling]
        return response

    def follow_objects(self,video_objects):
        #resp=self.comms.immediate_get(["ultrasonic_sensor","last_measurement"])

        #self.comms.set_intention(["ultrasonic_sensor","distance","GET"],None)
        #resp=self.comms.immediate_get(["ultrasonic_sensor","last_measurement"])
        resp=self.comms.get_state(["ultrasonic_sensor","last_measurement"])
        print("resp {}".format(resp))
        ret_objects=[]
        for x in video_objects:
            if (x["label"] != "sports ball" and x["label"] != "orange") or x["confidence"]<0.5:
                continue
            pos,wh=self.tagger.get_obj_loc_width(x)
            target_width=0.6
            #pixel_to_turn=-0.8
            pixel_to_turn=-0.4

            width_to_fb=1.0
            print("chasing {} at {} wh {}".format(x["label"],pos,wh))
            #time_scale=0.1
            #translation=[ width_to_fb*(target_width-wh[0]), 0, pixel_to_turn*pos[0],0 ]
            #translation=[ width_to_fb*(target_width-wh[0]), 0, pixel_to_turn*pos[0],0 ]
            fb=min(0.05/abs(pos[0]),1.0)
            translation=[ fb, 0, pixel_to_turn*pos[0],0 ]

            #translation[0]=0 #no fb
            print("translaation {}".format(translation))
            converted=self.interpret_translation(translation)
            print("converted {}".format(converted))
            #self.comms.set_intention( ["drive","translate","SET"],converted)
            self.comms.set( ["drive","translate"],converted)
            ret_objects.append(x)

            break
        return ret_objects

    def act(self):
        self.comms.update()
        self.handle_keys()
        ret=self.get_image()
        if self.last_last_image_timestamp==None or self.last_image_timestamp>self.last_last_image_timestamp:
            self.last_last_image_timestamp=self.last_image_timestamp
        else:
            print("repeat frame!")
        if ret is not None:
            video_objects=self.tagger.tag_objects(ret)
            self.tagger.log_tags(self.object_log_fname,time.time()-self.start_time)
            self.follow_objects(video_objects)
            ret=self.tagger.draw_bboxes(ret,video_objects)
            ret=self.tag_with_fps(ret)
            return ret
        return None
        #return self.get_image()
