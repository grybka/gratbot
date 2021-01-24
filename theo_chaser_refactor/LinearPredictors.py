import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random
import json
import sys,os,traceback

import logging
import numpy as np

from TimeSeries import TimeSeries,TrackData
from scipy.optimize import curve_fit

class TurnPredictor():
    def __init__(self):
        pass
    def load_from_dict(self,dat):
        pass
    def save_to_dict(self,dat):
        pass
    def predict_video_from_motion(self,turn):
        pass
    def predict_compass_from_motion(self,turn):
        pass
    def predict_turn_for_compass_angle(self,compass_angle):
        pass
    def get_max_turn_angle(self):
        pass

class LinearTurnPredictor(TurnPredictor):
    def __init__(self):
        self.video_turn_slope=2.68
        self.video_turn_slope_unc=0.05
        self.video_turn_inherent_unc=0.02
        self.compass_turn_slope=3.1
        self.compass_turn_slope_unc=0.1
        self.compass_turn_inherent_unc=0.1

    def load_from_dict(self,dat):
        try:
            self.video_turn_slope=dat["turn_predictor"]["video_turn_slope"]
            self.video_turn_slope_unc=dat["turn_predictor"]["video_turn_slope_unc"]
            self.compass_turn_slope=dat["turn_predictor"]["compass_turn_slope"]
            self.compass_turn_slope_unc=dat["turn_predictor"]["compass_turn_slope_unc"]
            self.compass_turn_inherent_unc=dat["turn_predictor"]["compass_turn_inherent_unc"]
        except:
            logging.warning("Unable to load turn predictor")

    def save_to_dict(self,dat):
        dat["turn_predictor"]={}
        dat["turn_predictor"]["video_turn_slope"]=self.video_turn_slope
        dat["turn_predictor"]["video_turn_slope_unc"]=self.video_turn_slope_unc
        dat["turn_predictor"]["compass_turn_slope"]=self.compass_turn_slope
        dat["turn_predictor"]["compass_turn_slope_unc"]=self.compass_turn_slope_unc
        dat["turn_predictor"]["compass_turn_inherent_unc"]=self.compass_turn_inherent_unc
        return dat


    def load_log_file(self,fname):
        f=open(fname,'r')
        headings=TimeSeries(title="Compass Heading",ylabel="Radians")
        dists=TimeSeries(title="Ultrasonic Distance",ylabel="Distance (m)")
        turn_commands=TimeSeries(title="Turn Commands",ylabel="Magnitude")
        fb_commands=TimeSeries(title="For/Bac Commands",ylabel="Magnitude")
        tracks={}
        first_timestamp=0
        for line in f.readlines():
            dat=json.loads(line)
            timestamp=dat["timestamp"]-first_timestamp
            if first_timestamp==0:
                first_timestamp=timestamp
                timestamp=0
            if "compass_heading" in dat:
                headings.append(timestamp,dat["compass_heading"])
            if "command" in dat:
                if dat["command"]["address"]==["drive","translate"]:
                    if dat["command"]["value"][0]!=0:
                        fb_commands.append(timestamp,dat["command"]["value"][0])
                    if dat["command"]["value"][2]!=0:
                        turn_commands.append(timestamp,dat["command"]["value"][2]*dat["command"]["value"][3])
            if "tracked_objects" in dat:
                for obj in dat["tracked_objects"]:
                    if obj["id"] not in tracks:
                        tracks[obj["id"]]=TrackData()
                    tracks[obj["id"]].append(timestamp,obj["xywh"],obj["last_update"],obj["label"])
            if "ultrasonic_sensor/last_measurement" in dat:
                dists.append(timestamp,dat["ultrasonic_sensor/last_measurement"]["average_distance"])
        return headings,dists,turn_commands,fb_commands,tracks

    def fit_compass_calibration(headings,turn_commands):
        turns_matching,headings_before,headings_after=headings.extract_bracketing_elements(turn_commands,after_time_min=0.5)
        delta_headings=TimeSeries(headings_before.times,min_angle_difference(headings_after.data,headings_before.data))

        def lin_fitfcn(x,m):
            return x*m

        popt, pcov = curve_fit(lin_fitfcn, turns_matching.data, delta_headings.data)
        sigma=np.sqrt(np.sum((lin_fitfcn(turns_matching.data,popt[0])-delta_headings.data)**2)/len(delta_headings))
        self.compass_turn_slope=np.asscalar(popt[0])
        self.compass_turn_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))
        self.compass_turn_inherent_unc=np.asscalar(sigma)

    def fit_compass_calibration_from_log(self,fname):
        headings,dists,turn_commands,fb_commands,tracks=self.load_log_file(fname)
        self.fit_compass_calibration(headings,turn_commands)

    def fit_vid_calibration(self,tracks,turn_commands):
        all_delta_vid=[]
        all_turn_mag=[]
        for key in tracks:
            xseries=TimeSeries(tracks[key].times,np.array(tracks[key].last_update)[:,0])
            turns_matching,trackx_before,trackx_after=xseries.extract_bracketing_elements(turn_commands,after_time_min=0.5)
            if len(turns_matching)==0:
                continue
            delta_vid=TimeSeries(trackx_before.times,min_angle_difference(trackx_after.data,trackx_before.data))
            all_delta_vid.extend(delta_vid.data)
            all_turn_mag.extend(turns_matching.data)
        all_turn_mag=np.array(all_turn_mag)
        all_delta_vid=np.array(all_delta_vid)

        def lin_fitfcn(x,m):
            return x*m

        popt, pcov = curve_fit(lin_fitfcn, all_turn_mag, all_delta_vid)
        sigma=np.sqrt(np.sum((lin_fitfcn(all_turn_mag,popt[0])-all_delta_vid)**2)/len(all_delta_vid))
        self.vid_turn_slope=np.asscalar(popt[0])
        self.vid_turn_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))
        self.vid_turn_inherent_unc=np.asscalar(sigma)

    def fit_vid_calibration_from_log(self,fname):
        headings,dists,turn_commands,fb_commands,tracks=self.load_log_file(fname)
        self.fit_compass_calibration(tracks,turn_commands)

    def predict_video_from_motion(self,turn):
        return turn*self.video_turn_slope, turn*self.video_turn_slope_unc

    def predict_compass_from_motion(self,turn):
        val=turn*self.compass_turn_slope
        unc=np.sqrt( (turn*self.compass_turn_slope_unc)**2+self.compass_turn_inherent_unc**2)
        return val,unc

    def predict_turn_for_compass_angle(self,compass_angle):
        val=compass_angle/self.compass_turn_slope
        val_unc=val*(self.compass_turn_slope_unc/self.compass_turn_slope)
        return val,val_unc

    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Angle Change")
        ax.grid()
        plt.show()

    def get_max_turn_angle(self):
        r,_=self.predict_compass_from_motion(1.0)
        return r

class LinearFBPredictor():

    def __init__(self):
        self.ultrasonic_motion_slope=0.395
        self.ultrasonic_motion_slope_unc=0.007
        self.ultrasonic_motion_inherent_unc=0.01

    def load_from_dict(self,dat):
        try:
            self.ultrasonic_motion_slope=dat["fb_predictor"]["ultrasonic_motion_slope"]
            self.ultrasonic_motion_slope_unc=dat["fb_predictor"]["ultrasonic_motion_slope_unc"]
            self.ultrasonic_motion_inherent_unc=dat["fb_predictor"]["ultrasonic_motion_inherent_unc"]
        except:
            logging.warning("Unable to load FB predictor info")

    def save_to_dict(self,dat):
        dat["fb_predictor"]={}
        dat["fb_predictor"]["ultrasonic_motion_slope"]=self.ultrasonic_motion_slope
        dat["fb_predictor"]["ultrasonic_motion_slope_unc"]=self.ultrasonic_motion_slope_unc
        dat["fb_predictor"]["ultrasonic_motion_inherent_unc"]=self.ultrasonic_motion_inherent_unc
        return dat



    def load_log_file(self,fname):
        #Newer
        f=open(fname,'r')
        headings=TimeSeries(title="Compass Heading",ylabel="Radians")
        dists=TimeSeries(title="Ultrasonic Distance",ylabel="Distance (m)")
        turn_commands=TimeSeries(title="Turn Commands",ylabel="Magnitude")
        fb_commands=TimeSeries(title="For/Bac Commands",ylabel="Magnitude")
        tracks={}
        first_timestamp=0
        for line in f.readlines():
            dat=json.loads(line)
            timestamp=dat["timestamp"]-first_timestamp
            if first_timestamp==0:
                first_timestamp=timestamp
                timestamp=0
            if "compass_heading" in dat:
                headings.append(timestamp,dat["compass_heading"])
            if "command" in dat:
                if dat["command"]["address"]==["drive","translate"]:
                    if dat["command"]["value"][0]!=0:
                        fb_commands.append(timestamp,dat["command"]["value"][0]*dat["command"]["value"][3])
                    if dat["command"]["value"][2]!=0:
                        turn_commands.append(timestamp,dat["command"]["value"][2]*dat["command"]["value"][3])
            if "tracked_objects" in dat:
                for obj in dat["tracked_objects"]:
                    if obj["id"] not in tracks:
                        tracks[obj["id"]]=TrackData()
                    tracks[obj["id"]].append(timestamp,obj["xywh"],obj["last_update"],obj["label"])
            if "ultrasonic_sensor/last_measurement" in dat:
                dists.append(timestamp,dat["ultrasonic_sensor/last_measurement"]["average_distance"])
        return headings,dists,turn_commands,fb_commands,tracks

    def fit_ultra_calibration(self,fb_commands,dists):
        fb_matching,dist_before,dist_after=dists.extract_bracketing_elements(fb_commands,after_time_min=0.55)
        delta_dist=TimeSeries(dist_before.times,dist_before.data-dist_after.data)

        def lin_fitfcn(x,m):
            return x*m
        popt, pcov = curve_fit(lin_fitfcn, fb_matching.data, delta_dist.data)
        sigma=np.sqrt(np.sum((lin_fitfcn(fb_matching.data,popt[0])-delta_dist.data)**2)/len(delta_dist.data))
        self.ultrasonic_motion_slope=np.asscalar(popt[0])
        self.ultrasonic_motion_slope_unc=np.asscalar(np.sqrt(pcov[0][0]))
        self.ultrasonic_motion_inherent_unc=np.asscalar(sigma)
        return fb_matching,delta_dist

    def fit_ultra_calibration_from_log(self,fname):
        headings,dists,turn_commands,fb_commands,tracks=self.load_log_file(fname)
        return self.fit_ultra_calibration(fb_commands,dists)


    def predict_ultrasonic_from_motion(self,fb):
        return fb*self.ultrasonic_motion_slope,np.sqrt((fb*self.ultrasonic_motion_slope_unc)**2+self.ultrasonic_motion_inherent_unc**2)

    def predict_motion_from_ultrasonic(self,dist):
        val=dist/self.ultrasonic_motion_slope
        val_unc=val*(self.ultrasonic_motion_slope_unc/self.ultrasonic_motion_slope)
        return val,val_unc

    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Angle Change")
        ax.grid()
        plt.show()

    def get_max_fb_dist(self):
        r,_=self.predict_ultrasonic_from_motion(1.0)
        return r
