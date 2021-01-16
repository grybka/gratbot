
import time
from GratbotUV4LConnection import GratbotUV4LConnection
from VisualTracker import VisualTracker
import threading
import numpy as np
from scipy.optimize import curve_fit
import json
import sys,os,traceback
from scipy.optimize import curve_fit
import yaml
import matplotlib
import matplotlib.pyplot as plt
from GratbotMap import GratbotMap

class TurnPredictor():
    def __init__(self):
        self.video_turn_slope=1
        self.video_turn_slope_unc=1
        self.compass_turn_slope=1
        self.compass_turn_slope_unc=1

    def load_from_dict(self,dat):
        self.video_turn_slope=dat["turn_predictor"]["video_turn_slope"]
        self.video_turn_slope_unc=dat["turn_predictor"]["video_turn_slope_unc"]
        self.compass_turn_slope=dat["turn_predictor"]["compass_turn_slope"]
        self.compass_turn_slope_unc=dat["turn_predictor"]["compass_turn_slope_unc"]

    def save_to_dict(self,dat):
        dat["turn_predictor"]={}
        dat["turn_predictor"]["video_turn_slope"]=self.video_turn_slope
        dat["turn_predictor"]["video_turn_slope_unc"]=self.video_turn_slope_unc
        dat["turn_predictor"]["compass_turn_slope"]=self.compass_turn_slope
        dat["turn_predictor"]["compass_turn_slope_unc"]=self.compass_turn_slope_unc
        return dat

    def fit_calibration(self,turns,vids,comps):
        def lin_fitfcn(x,m):
            return x*m
        popt, pcov = curve_fit(lin_fitfcn, turns, vids)
        self.video_turn_slope=np.asscalar(popt[0])
        self.video_turn_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))

        test_x=np.linspace(-1,1,100)
        test_y=self.predict_video_from_motion(test_x)
        fig,ax=plt.subplots()
        ax.plot(turns,vids,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Video Change")
        ax.grid()
        plt.show()


        popt, pcov = curve_fit(lin_fitfcn, turns, comps)
        self.compass_turn_slope=np.asscalar(popt[0])
        self.compass_turn_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))

        test_x=np.linspace(-1,1,100)
        test_y=self.predict_compass_from_motion(test_x)
        fig,ax=plt.subplots()
        ax.plot(turns,comps,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Compass Change")
        ax.grid()
        plt.show()

    def predict_video_from_motion(self,turn):
        return turn*self.video_turn_slope

    def predict_compass_from_motion(self,turn):
        return turn*self.compass_turn_slope

    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Angle Change")
        ax.grid()
        plt.show()

class FBPredictor():

    def __init__(self):
        self.ultrasonic_motion_slope=1
        self.ultrasonic_motion_slope_unc=1

    def load_from_dict(self,dat):
        if "fb_predictor" not in dat:
            return False
        self.video_turn_slope=dat["fb_predictor"]["ultrasonic_motion_slope"]
        self.video_turn_slope_unc=dat["fb_predictor"]["ultrasonic_motion_slope_unc"]

    def save_to_dict(self,dat):
        dat["fb_predictor"]={}
        dat["fb_predictor"]["ultrasonic_motion_slope"]=self.ultrasonic_motion_slope
        dat["fb_predictor"]["ultrasonic_motion_slope_unc"]=self.ultrasonic_motion_slope_unc
        return dat

    def fit_calibration(self,fbs,ults):
        def lin_fitfcn(x,m):
            return x*m
        popt, pcov = curve_fit(lin_fitfcn, fbs, ults)
        self.ultrasonic_motion_slope=np.asscalar(popt[0])
        self.ultrasonic_motion_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))

        test_x=np.linspace(-1,1,100)
        test_y=self.predict_ultrasonic_from_motion(test_x)
        fig,ax=plt.subplots()
        ax.plot(fbs,ults,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="FB Magnitude",ylabel="Ultrasonic Change")
        ax.grid()
        plt.show()

    def predict_ultrasonic_from_motion(self,fb):
        return fb*self.ultrasonic_motion_slope

    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Angle Change")
        ax.grid()
        plt.show()

class GratbotSensorFusion():
    def __init__(self):
        self.video=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")
        self.gratbot_state={}
        self.last_video_frame_lock=threading.Lock()
        self.last_video_frame=None
        self.user_frame_lock=threading.Lock()
        self.user_frame=None
        self.short_term_memory={}
        self.turn_predictor=TurnPredictor()
        self.fb_predictor=FBPredictor()
        #calibrations here
        #should I keep them in a yaml file or something?
        #self.b_field_correction=np.array([242.8,-238.2,-997.7]) #next to motor
        self.b_field_correction=np.array([-5.63,118.74,-58.58]) #new position
        #object perception
        self.tracker=VisualTracker()
        self.save_updates=False
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        self.save_filename = "logs/sensor_log_{}.txt".format(self.start_timestr)
        self.save_lock=threading.Lock()
        self.map=GratbotMap()

    def save_config(self,fname):
        output_config={}
        output_config["b_field_correction"]=self.b_field_correction
        output_config=self.turn_predictor.save_to_dict(output_config)
        output_config=self.fb_predictor.save_to_dict(output_config)
        print("output config: {}".format(output_config))
        f=open(fname,'w')
        f.write(yaml.dump(output_config))
        f.close()

    def load_config(self,fname):
        f=open(fname,'r')
        cfg=yaml.load(f,Loader=yaml.FullLoader)
        f.close()
        self.b_field_correction=cfg["b_field_correction"]
        self.turn_predictor.load_from_dict(cfg)
        self.fb_predictor.load_from_dict(cfg)

    def update(self,comms):
        resp=comms.update()
        self.gratbot_state.update(resp)
        self.gratbot_state["compass_heading"]=self.get_compass_heading()
        self.map.update_with_compass(self.get_compass_heading())
        if self.save_updates:
            resp["compass_heading"]=self.get_compass_heading()
            resp["timestamp"]=time.time()
            self.save_lock.acquire()
            f=open(self.save_filename,"a")
            f.write(json.dumps(resp)+"\n")
            f.close()
            self.save_lock.release()

    def update_memory(self,key,value):
        #print("resp was {}".format(resp))
        self.short_term_memory[key]=value
        if self.save_updates:
            item={key: value}
            item["timestamp"]=time.time()
            self.save_lock.acquire()
            f=open(self.save_filename,"a")
            f.write(json.dumps(item)+"\n")
            f.close()
            self.save_lock.release()

    def update_video(self):
        self.last_video_frame_lock.acquire()
        mystery_status,self.last_video_frame=self.video.get_latest_frame()
        self.last_video_frame_lock.release()
        self.tracker.update(self.last_video_frame)
        self.user_frame_lock.acquire()
        self.user_frame=self.last_video_frame
        self.user_frame=self.tracker.draw_bboxes(self.user_frame)
        if self.save_updates:
            item={"tracked_objects": self.tracker.serialize_tracks()}
            item["timestamp"]=time.time()
            self.save_lock.acquire() #threadlock danger
            f=open(self.save_filename,"a")
            f.write(json.dumps(item)+"\n")
            f.close()
            self.save_lock.release()
        self.user_frame_lock.release()

    def get_user_display(self):
        ret={}
        #self.last_video_frame_lock.acquire()
        #the_frame=self.last_video_frame
        #self.last_video_frame_lock.release()
        self.user_frame_lock.acquire()
        ret["vision"]=self.user_frame
        #the_frame=self.user_frame
        self.user_frame_lock.release()
        ret["map"]=self.map.map_image()
        return ret

    def stop(self):
        self.video.stop()

    def save_state(self,fname="gratbot_sensors.yaml"):
        state["b_field_correction"]=self.b_field_correction.tolist()
        with open(fname, 'w') as f:
            data = yaml.dump(state, f)
            f.close()

    def load_state(self,fname="gratbot_sensors.yaml"):
        with open(fname) as f:
            data = yaml.load_all(f, Loader=yaml.FullLoader)
            if "b_field_correction" in data:
                self.b_field_correction=np.array(data["b_field_correction"])
            f.close()

    #specialized functions here
    def get_corrected_bfield(self):
        if "magnetometer/b_field" in self.gratbot_state:
            return np.array(self.gratbot_state["magnetometer/b_field"])-self.b_field_correction
        return np.zeros(3)

    def get_compass_heading(self):
        b=self.get_corrected_bfield()
        return np.arctan2(b[0],b[1])

    #If I want to turn X degrees, how much do I send to my turn command?

    #Given an object on my screen, what angle do I thin kit is?
    #So a device that turns my objects into an angle and a distance

    #step 1, calibrate turn-to-angle
    #step 2, calibrate turn to center object
    #step 3, I can use those two to create an object to angle mapping


    #for motion
    def send_command(self,comms,address,value):
        comms.set(address,value)
        if self.save_updates:
            item={"command": {"address": address,"value": value}}
            item["timestamp"]=time.time()
            self.save_lock.acquire()
            f=open(self.save_filename,"a")
            f.write(json.dumps(item)+"\n")
            f.close()
            self.save_lock.release()

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
