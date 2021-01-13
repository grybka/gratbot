from GratbotUV4LConnection import GratbotUV4LConnection
from VisualTracker import VisualTracker
import threading
import numpy as np
from scipy.optimize import curve_fit

class GratbotSensorFusion():
    def __init__(self):
        self.video=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")
        self.gratbot_state={}
        self.last_video_frame_lock=threading.Lock()
        self.last_video_frame=None
        self.user_frame_lock=threading.Lock()
        self.user_frame=None
        self.short_term_memory={}
        #calibrations here
        #should I keep them in a yaml file or something?
        #self.b_field_correction=np.array([242.8,-238.2,-997.7]) #next to motor
        self.b_field_correction=np.array([-5.63,118.74,-58.58]) #new position
        #object perception
        self.tracker=VisualTracker()

    def update(self,comms):
        resp=comms.update()
        self.gratbot_state.update(resp)
        self.gratbot_state["compass_heading"]=self.get_compass_heading()
        #print("resp was {}".format(resp))

    def update_video(self):
        self.last_video_frame_lock.acquire()
        mystery_status,self.last_video_frame=self.video.get_latest_frame()
        self.last_video_frame_lock.release()
        self.tracker.update(self.last_video_frame)
        self.user_frame_lock.acquire()
        self.user_frame=self.last_video_frame
        self.user_frame=self.tracker.draw_bboxes(self.user_frame)
        self.user_frame_lock.release()

    def get_user_display(self):
        #self.last_video_frame_lock.acquire()
        #the_frame=self.last_video_frame
        #self.last_video_frame_lock.release()
        self.user_frame_lock.acquire()
        the_frame=self.user_frame
        self.user_frame_lock.release()
        return the_frame

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
