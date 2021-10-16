

from Gyrus import ThreadedGyrus
import logging
import time

import numpy as np
import cv2 as cv
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
from scipy.signal import resample
from scipy.fftpack import shift

from underpinnings.MotionCorrection import MotionCorrectionRecord
#combine information from audio and trackers to
#generate important directions nearby

#if I can see something, and it has my attention, then it should
#dominate my direction

#if I can hear something and it has my attention, then it should dominate my direction

class PeripersonalSpaceGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.display=display
        self.n_direction_neurons=32
        self.direction_neurons=np.zeros(self.n_direction_neurons)
        self.neuron_thetas=np.arange(self.n_direction_neurons)*2*np.pi/self.n_direction_neurons

        #for weights
        self.attention_in_fov=False

        #for decay
        ## TODO: if I recieve new information, decay should be stronger
        ##if i I have no new information, decay should be weaker
        self.decay_timescale=10.0
        self.decay_update_period=0.10
        self.next_decay_update=0
        self.decay_amount=np.exp(-self.decay_update_period/self.decay_timescale   )
        self.last_message_emission=0

        #for handling turning
        self.motion_corrector=MotionCorrectionRecord()
        self.latest_timestamp=0
        self.rotation_update_period=0.05
        self.last_rotation_update=0

        #for drawing
        self.next_draw_update=0
        self.draw_update_period=0.5

    def get_keys(self):
        return ["tracks","rotation_vector","clock_pulse","track_object_pairing","word_heard"]

    def get_name(self):
        return "PeripersonalSpaceGyrus"

    def emit_heading_message(self):
        self.broker.publish({"timestamp": time.time(),"peripersonal_direction": self.direction_neurons},["peripersonalspace"])
        self.last_message_emission=time.time()

    def get_excited_direction(self,mean,sigma):
        deltas=(self.neuron_thetas-mean+ np.pi) % (2 * np.pi) - np.pi
        excites=np.exp( -deltas*deltas/(2*sigma*sigma) )
        excites=excites-np.sum(excites)/len(excites)
        excites=excites/np.max(excites)
        return excites

    def excite_direction(self,mean,sigma,weight):
        deltas=(self.neuron_thetas-mean+ np.pi) % (2 * np.pi) - np.pi
        excites=weight*np.exp( -deltas*deltas/(2*sigma*sigma) )
        excites=excites-np.sum(excites)/len(excites)
        self.direction_neurons+=excites
        self.direction_neurons=np.clip(self.direction_neurons,-1,1)
        self.emit_heading_message()
        return excites

    def do_rotation_correction(self):
        new_time=self.motion_corrector.get_latest_timestamp()
        turn,pitch=self.motion_corrector.get_rotation_between(self.latest_timestamp,new_time)
        #cycle the neurons
        if self.latest_timestamp!=0:
            if np.abs(turn)>0.05:
                self.direction_neurons=np.clip(shift(self.direction_neurons,turn),-1,1)
                self.emit_heading_message()
        self.latest_timestamp=new_time


    def read_message(self,message):
        if 'packets' in message:
            self.motion_corrector.read_message(message)
            if time.time()-self.last_rotation_update>self.rotation_update_period:
                self.do_rotation_correction()
                self.last_rotation_update=time.time()
        if 'tracks' in message:
            self.handle_tracks(message["tracks"],message["image_timestamp"])
        if 'word_heard' in message:
            self.handle_word(message["word_heard"])
        if 'clock_pulse' in message:
            if time.time()>self.next_draw_update:
                self.draw_object_map()
                self.next_draw_update=time.time()+self.draw_update_period
            if time.time()>self.next_decay_update:
                self.direction_neurons=self.decay_amount*self.direction_neurons
                self.next_decay_update=time.time()+self.decay_update_period
                if not self.attention_in_fov:
                    self.fov_is_boring()
            if time.time()-self.last_message_emission>0.5:
                self.emit_heading_message()

    def fov_is_boring(self):
        self.boring_seconds=4
        self.direction_neurons[abs(self.neuron_thetas)<(2*np.pi*25/360)]-=self.decay_update_period/self.boring_seconds
        self.direction_neurons=np.clip(self.direction_neurons,-1,1)

    def track_center_to_angle(self,track_center,image_timestamp):
        x=track_center[0]-0.5
        focal_length=1.0/(2*np.tan(np.pi*51.5/360)) #in fraction of image
        angle=x/focal_length
        turn_mag,pitch_mag=self.motion_corrector.get_rotation_between(image_timestamp,self.latest_timestamp)
        return angle+turn_mag
        #return angle-turn_mag

    def handle_word(self,word_heard):
        heading=word_heard["heading"]
        sigma=(20.0/360)*(2*np.pi) #uncertainty in sigma 2 degrees
        weight=2.0
        if word_heard["speaker"][0][0]=="gray":
            weight=3.0
        elif word_heard["word"][0][0]=="unknown":
            weight=0.2
        self.excite_direction(heading,sigma,weight)

    def get_track_attention(self,track,image_timestamp):
        if track["info"]=="LOST" or track["info"]=="EXITED":
            return 0
        elif track["center"][0]<0 or track["center"][0]>1:
            return 0
        elif track["label"]=="face":
            return 0.9
        return 0

    def handle_tracks(self,tracks,image_timestamp):
        excitation_sum=np.zeros(len(self.neuron_thetas))
        weight_sum=0
        for track in tracks:
            weight=self.get_track_attention(track,image_timestamp)
            if weight==0:
                continue
            angle=self.track_center_to_angle(track["center"],image_timestamp)
            sigma=(2.0/360)*(2*np.pi) #uncertainty in sigma 2 degrees
            excitation_sum+=weight*self.get_excited_direction(angle,sigma)
            weight_sum+=weight
        if weight_sum==0:
            return
        if weight_sum>1:
            self.attention_in_fov=True
            excitation_sum=excitation_sum/weight_sum
            weight_sum=1
        #self.direction_neurons=self.direction_neurons*(1-weight_sum)+excitation_sum
        self.direction_neurons=np.clip(self.direction_neurons+excitation_sum,-1,1)


    def draw_object_map(self):

        downsampled=np.clip(resample(self.direction_neurons,12),-1,1)
        downsampled=np.roll(downsampled,-3) #because the zero is different

        my_rgb=[]
        for i in range(12):
            val=int(100*downsampled[i])
            if val>0:
                my_rgb.append([val,val,val])
            else:
                val=abs(val)
                my_rgb.append([0,0,val])
        self.broker.publish({"led_command":{"rgb_brightness": my_rgb}},"led_command")
        xsize=320
        ysize=240
        step=ysize//4
        toshow=np.zeros([ysize,xsize,3],np.uint8)
        #cv.arrowedLine(toshow,(xsize//2,ysize//2),(xsize//2,ysize//2),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        cv.arrowedLine(toshow,(xsize//2,ysize//2),(xsize//2+20,ysize//2),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        intensities=128+128*np.clip(self.direction_neurons,-1,1)
        for i in range(self.n_direction_neurons):
            theta=self.neuron_thetas[i]
            x=int(xsize//2+step*np.cos(theta))
            y=int(ysize//2+step*np.sin(theta))
            c=int(intensities[i])
            #logger.debug(c)
            cv.circle(toshow,(x,y),3,(c,c,c),4)
        self.display.update_image("peripersonalspace",toshow)
