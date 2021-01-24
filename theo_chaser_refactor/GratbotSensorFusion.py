
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
import logging
from LinearPredictors import TurnPredictor,LinearTurnPredictor,LinearFBPredictor



class FBPredictor():
    def __init__(self):
        pass
    def load_from_dict(self,dat):
        pass
    def save_to_dict(self,dat):
        pass
    def predict_ultrasonic_from_motion(self,fb):
        pass
    def predict_motion_from_ultrasonic(self,dist):
        pass
    def get_max_fb_dist(self):
        pass

class GratbotSensorFusion():
    def __init__(self):
        self.video=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")
        self.gratbot_state={}
        self.last_video_frame_lock=threading.Lock()
        self.last_video_frame=None
        self.user_frame_lock=threading.Lock()
        self.user_frame=None
        self.short_term_memory={}
        self.turn_predictor=LinearTurnPredictor()
        self.fb_predictor=LinearFBPredictor()
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

        #sensor warning times
        self.compass_off_until=time.time()
        self.motor_compass_disruption_time=0.16 #seconds
        self.video_off_until_time=time.time()
        #self.motor_video_disruption_time=0.16 #seconds
        self.motor_video_disruption_time=0.35 #seconds, saccade time

    def save_config(self,fname):
        output_config={}
        output_config["b_field_correction"]=self.b_field_correction.tolist()
        output_config=self.turn_predictor.save_to_dict(output_config)
        output_config=self.fb_predictor.save_to_dict(output_config)
        print("output config: {}".format(output_config))
        f=open(fname,'w')
        f.write(yaml.dump(output_config))
        f.close()

    def load_config(self,fname):
        try:
            f=open(fname,'r')
            cfg=yaml.load(f,Loader=yaml.FullLoader)
            f.close()
            self.b_field_correction=np.array(cfg["b_field_correction"])
            self.turn_predictor.load_from_dict(cfg)
            self.fb_predictor.load_from_dict(cfg)
        except:
            logging.warning("Unable to load config file, starting from scratch")

    def update(self,comms):
        resp=comms.update()
        self.gratbot_state.update(resp)
        if "magnetometer/b_field" in resp:
            if time.time()>self.compass_off_until:
                self.gratbot_state["compass_heading"]=self.get_compass_heading()
                self.map.update_pose_with_compass(self.get_compass_heading())
            else:
                pass # do not update compass if it has been disabled due to motion
        if "ultrasonic_sensor/last_measurement" in resp:
            self.map.update_map_with_ultrasonic(resp["ultrasonic_sensor/last_measurement"]["average_distance"],
resp["ultrasonic_sensor/last_measurement"]["stdev_distance"])
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
        if self.last_video_frame.shape!=(480,640,3):
            logging.warning("warning video of wrong shape, skipping frame shape was {}".format(self.last_video_frame.shape))
            return
        if time.time()<self.video_off_until_time:
            return
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


    #for motio
    def send_command_turn_angle(self,comms,turn_angle):
        #print("command to turn {} degrees".format(360*turn_angle/(2*np.pi)))
        turn_speed=0.6
        turn_mag,turn_mag_unc=self.turn_predictor.predict_turn_for_compass_angle(turn_angle)
        if abs(turn_mag)>1: #turn as far as you can but no more
            print("requesting turn too far")
            turn_mag=np.sign(turn_mag)*1.0
        #feed it backwards so I know my uncertainty
        angle,angle_unc=self.turn_predictor.predict_compass_from_motion(turn_mag)
        translation=[0,0,turn_mag]
        self.short_term_memory["last_translation"]=translation
        #Warn compass will be disrupted
        self.compass_off_until=time.time()+self.motor_compass_disruption_time
        #TODO warn video will be disrupted
        self.video_off_until_time=time.time()+self.motor_video_disruption_time

        #do actual turn
        self.send_command(comms, ["drive","translate"],[0,0,np.sign(turn_mag)*turn_speed,abs(turn_mag)/turn_speed])

        #adjust pose belief
        cov=np.zeros([3,3])
        cov[2][2]=angle_unc**2
        #print("informing pose to change to turn {} degrees".format(360*angle/(2*np.pi)))
        self.map.change_pose_with_offset(np.array([0,0,angle]),cov)

        #adjust video belief
        #self.send_command(comms, ["drive","translate"],self.interpret_translation(translation))
        vid_x,vid_x_unc=self.turn_predictor.predict_video_from_motion(turn_mag)
        self.tracker.adjust_all_tracks(self,vid_x,0,0,0,vid_x_unc,0,0,0)

    def send_command_forward_meters(self,comms,dist):
        #print("received command to go forward {}".format(dist))
        fb,fb_unc=self.fb_predictor.predict_motion_from_ultrasonic(dist)
        if abs(fb)>1:
            print("Trying to go too far, {}".format(fb))
            fb=np.sign(fb)*1.0
        #print("thats a translation of {}".format(fb))
        newdist,newdist_unc=self.fb_predictor.predict_ultrasonic_from_motion(fb)
        translation=[fb,0,0]
        self.short_term_memory["last_translation"]=translation
        #Warn compass will be disrupted
        self.compass_off_until=time.time()+self.motor_compass_disruption_time
        #TODO warn video will be disrupted
        self.video_off_until_time=time.time()+self.motor_video_disruption_time

        #adjust pose belief
        dx=-dist*np.cos(self.map.pose[2])
        dy=dist*np.sin(self.map.pose[2])
        dxdr=-np.cos(self.map.pose[2])
        dydr=np.sin(self.map.pose[2])
        dxdtheta=dist*np.sin(self.map.pose[2])
        dydtheta=dist*np.cos(self.map.pose[2])
        sigma_r=newdist_unc*newdist_unc
        sigma_theta=self.map.pose_covariance[2][2]
        sigma_xx=sigma_r*(dxdr*dxdr)+sigma_theta*(dxdtheta*dxdtheta)
        sigma_yy=sigma_r*(dydr*dydr)+sigma_theta*(dydtheta*dydtheta)
        sigma_xy=sigma_r*(dxdr*dydr)+sigma_theta*(dydr*dydtheta)
        cov=np.zeros([3,3])
        cov[0][0]=sigma_xx
        cov[1][1]=sigma_yy
        cov[0][1]=sigma_xy
        cov[1][0]=sigma_xy
        cov[2][2]=0.0001 #hardcoding some turn drift
        print("updating pose to go forward {},{}".format(dx,dy))
        self.map.change_pose_with_offset(np.array([dx,dy,0]),cov)

        #TODO adjust video belief (not really necessary I think)

        self.send_command(comms, ["drive","translate"],self.interpret_translation(translation))

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
