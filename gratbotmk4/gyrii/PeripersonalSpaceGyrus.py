

from Gyrus import ThreadedGyrus
import logging
import time

import numpy as np
import cv2 as cv
logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
logger.setLevel(logging.INFO)
from underpinnings.direction_neurons import direction_neuron_config,distance_neuron_config

from underpinnings.MotionCorrection import MotionCorrectionRecord
#combine information from audio and trackers to
#generate important directions nearby

#if I can see something, and it has my attention, then it should
#dominate my direction

#if I can hear something and it has my attention, then it should dominate my direction

# gyrus_config: { target: PeripersonalSpaceGyrus,
#              attention: {speaker_attention: [],
#                          speaker_suppression: [],
#                          word_attention: [],
#                          word_suppression: [],
#                          image_label_attention: [],
#                          image_label_suppression: [],
#                          track_id_attention: [],
#                          track_id_suppression: []}}}

class PeripersonalSpaceGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.display=display
        #self.n_direction_neurons=32
        #self.n_pitch_neurons=5
        #self.bottom_pitch=-(30/360)*2*np.pi
        #self.top_pitch=(60/360)*2*np.pi

        #self.direction_neurons=np.zeros(self.n_direction_neurons)
        #self.direction_neurons=np.zeros([self.n_direction_neurons,self.n_pitch_neurons])
        self.direction_neurons=direction_neuron_config.get_blank_neurons()
        self.distance_neurons=distance_neuron_config.get_blank_neurons()
        #self.neuron_thetas=np.arange(self.n_direction_neurons)*2*np.pi/self.n_direction_neurons
        #self.neuron_pitches=np.linspace(self.bottom_pitch,self.top_pitch,self.n_pitch_neurons)

        #for attention
        self.speaker_attention=["gray"]
        self.speaker_suppression=["background"]
        self.word_attention=["robot"]
        self.word_suppression=["background_noise"]
        self.image_label_attention=["face"]
        self.track_id_attention=[]

        #for weights
        self.fov_attention_span=4 #in seconds
        self.last_attention_in_fov=0

        #for decay
        ## TODO: if I recieve new information, decay should be stronger
        ##if i I have no new information, decay should be weaker
        self.decay_timescale=5.0
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
        return ["tracks","rotation_vector","clock_pulse","track_object_pairing","word_heard","gyrus_config"]

    def get_name(self):
        return "PeripersonalSpaceGyrus"

    def emit_heading_message(self):
        self.broker.publish({"timestamp": time.time(),"peripersonal_direction": self.direction_neurons,"peripersonal_distance": self.distance_neurons},["peripersonalspace"])
        self.last_message_emission=time.time()

#    def get_excited_direction(self,mean,sigma):
#        deltas=(self.neuron_thetas-mean+ np.pi) % (2 * np.pi) - np.pi
#        excites=np.exp( -deltas*deltas/(2*sigma*sigma) )
#        excites=excites-np.sum(excites)/len(excites)
#        excites=excites/np.max(excites)
#        return excites


#    def get_excited_direction_with_pitch(self,mean,sigma,pitch_mean,pitch_sigma):
#        deltas=(self.neuron_thetas-mean+ np.pi) % (2 * np.pi) - np.pi
#        deltas_pitch=(self.neuron_pitches-pitch_mean)
#        #logger.debug("deltas pitch {}".format(deltas_pitch))
#        #logger.debug("sigma, sigma_pitch {},{}".format(sigma,pitch_sigma))
#        excites_theta=np.exp( -deltas*deltas/(2*sigma*sigma) )
#        excites_pitch=np.exp( -deltas_pitch*deltas_pitch/(2*pitch_sigma*pitch_sigma))
#        #logger.debug("excites theta {}".format(excites_theta))
#        #logger.debug("excites pitch {}".format(excites_theta))
#        excites=np.outer(excites_theta,excites_pitch)
#        excites=excites-np.sum(excites)/len(excites)
#        excites=excites/np.max(excites)
#        #logger.debug("excitings {}".format(excites))
#        return excites

#    def excite_direction(self,mean,sigma,weight):
#        deltas=(self.neuron_thetas-mean+ np.pi) % (2 * np.pi) - np.pi
#        excites=weight*np.exp( -deltas*deltas/(2*sigma*sigma) )
#        excites=excites-np.sum(excites)/len(excites)
#        for i in range(self.n_pitch_neurons):
#            self.direction_neurons[:,i]+=excites
#            self.direction_neurons[:,i]=np.clip(self.direction_neurons[:,i],-1,1)
#        self.emit_heading_message()
#        return excites

    def do_rotation_correction(self):
        new_time=self.motion_corrector.get_latest_timestamp()
        turn,pitch=self.motion_corrector.get_rotation_between(self.latest_timestamp,new_time)
        #cycle the neurons
        if self.latest_timestamp!=0:
            if np.abs(turn)>0.05:
                direction_neuron_config.rotate(turn,self.direction_neurons)
                #self.direction_neurons=np.clip(shift(self.direction_neurons,turn),-1,1)
                #for i in range(self.n_pitch_neurons):
                #    self.direction_neurons[:,i]=np.clip(shift(self.direction_neurons[:,i],turn),-1,1)
                self.emit_heading_message()
        self.latest_timestamp=new_time


    def read_message(self,message):
        if "gyrus_config" in message and message["gyrus_config"]["target"]==self.get_name():
            m=message["gyrus_config"]
            if "speaker_attention" in m:
                self.speaker_attention=m["speaker_attention"]
            if "speaker_suppression" in m:
                self.speaker_suppression=m["speaker_suppression"]
            if "word_attention" in m:
                self.word_attention=m["word_attention"]
            if "word_suppression" in m:
                self.word_suppression=m["word_suppression"]
            if "image_label_attention" in m:
                self.image_label_attention=m["image_label_attention"]
            if "image_label_suppression" in m:
                self.image_label_suppression=m["image_label_suppression"]
            if "track_id_attention" in m:
                self.track_id_attention=m["track_id_attention"]
            if "track_id_suppression" in m:
                self.track_id_suppression=m["track_id_suppression"]
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
            if time.time()-self.last_attention_in_fov>self.fov_attention_span:
                self.fov_is_boring()
            if time.time()-self.last_message_emission>0.5:
                self.emit_heading_message()

    def fov_is_boring(self):
        self.boring_seconds=4
        my_pitch=self.motion_corrector.get_pitch()
        theta_range=np.logical_or(abs(direction_neuron_config.neuron_thetas)<(2*np.pi*25/360),abs(direction_neuron_config.neuron_thetas)>(2*np.pi-2*np.pi*25/360))
        #logger.debug("pitch deltas {}".format(self.neuron_pitches-my_pitch))
        pitch_range=abs(direction_neuron_config.neuron_pitches-my_pitch)<(2*np.pi*25/360)
        indices=np.outer(theta_range,pitch_range)
        self.direction_neurons[indices]-=self.decay_update_period/self.boring_seconds
        self.direction_neurons[indices]=np.clip(self.direction_neurons[indices],-1,1)
        self.distance_neurons-=self.decay_update_period/self.boring_seconds
        self.distance_neurons=np.clip(self.distance_neurons,-1,1)
                #for i in range(self.n_pitch_neurons):
            #self.direction_neurons[abs(self.neuron_thetas)<(2*np.pi*25/360),i]-=self.decay_update_period/self.boring_seconds
            #self.direction_neurons[abs(self.neuron_thetas)>(2*np.pi-2*np.pi*25/360),i]-=self.decay_update_period/self.boring_seconds
        #    self.direction_neurons[theta_range]-=self.decay_update_period/self.boring_seconds
        #    self.direction_neurons[:,i]=np.clip(self.direction_neurons[:,i],-1,1)
        #logger.debug("bored now {}".format(self.direction_neurons))

    def track_center_to_angle(self,track_center,image_timestamp):
        logger.debug("track center {}".format(track_center))
        x=track_center[0]-0.5
        focal_length=1.0/(2*np.tan(np.pi*51.5/360)) #in fraction of image
        angle=x/focal_length
        turn_mag,pitch_mag=self.motion_corrector.get_rotation_between(image_timestamp,self.latest_timestamp)
        return angle+turn_mag
        #return angle-turn_mag

    def track_center_to_pitch(self,track_center,image_timestamp):
        y=-(track_center[1]-0.5)
        focal_length=1.0/(2*np.tan(np.pi*51.5/360)) #in fraction of image
        my_pitch=self.motion_corrector.get_pitch()
        track_pitch=y/focal_length
        pitch=track_pitch+my_pitch
        #logger.debug("my pitch {}".format(my_pitch))
        #logger.debug("track pitch {}".format(track_pitch))
        #logger.debug("total taregt pitch {}".format(pitch))
        return pitch

    def handle_word(self,word_heard):
        heading=word_heard["heading"]
        sigma=(20.0/360)*(2*np.pi) #uncertainty in sigma 2 degrees

        speaker=word_heard["speaker"][0][0]
        word=word_heard["word"][0][0]

        weight=0.5
        if word in self.speaker_attention:
            weight*=2
        if word in self.speaker_suppression:
            weight*=0.2
        if word in self.word_attention:
            weight*=2
        if word in self.word_suppression:
            weight*=0.2

        pitch=20*2*np.pi/360 #20 degrees above horizon
        sigma_pitch=20*2*np.pi/360
        #excitation=self.get_excited_direction_with_pitch(heading,sigma,pitch,sigma_pitch)
        excitation=direction_neuron_config.get_excitement(heading,sigma,pitch,sigma_pitch)
        logger.debug("excitation is {}".format(excitation))
        self.direction_neurons+=excitation
        #self.direction_neurons=np.clip(self.direction_neurons,-1,1)
        self.rescale()
        self.emit_heading_message()
        #self.excite_direction(heading,sigma,weight)

    def get_track_attention(self,track,image_timestamp):
        #if track['info']=="EXITED":
        #    return 0 #commenting out because I need to follow things off edge
        if track["info"]=="LOST":
            return 0
        elif track["center"][0]<0 or track["center"][0]>1:
            return 0
        my_attention=0.5
        if track["label"] in self.image_label_attention:
            my_attention*=2
        if track["id"] in self.track_id_attention:
            my_attention*=4
        return my_attention

    def rescale(self):
        #the choice is between clipping saturated values, or rescaling the whole thing
        #rescaling
        max_val=np.max(self.direction_neurons)
        if max_val>1:
            self.direction_neurons=np.clip(self.direction_neurons/max_val,-1,1)
        else:
            self.direction_neurons=np.clip(self.direction_neurons,-1,1)
        max_val=np.max(self.distance_neurons)
        if max_val>1:
            self.distance_neurons=np.clip(self.distance_neurons/max_val,-1,1)
        else:
            self.distance_neurons=np.clip(self.distance_neurons,-1,1)

    def handle_tracks(self,tracks,image_timestamp):
        #excitation_sum=np.zeros(len(self.neuron_thetas))
        #excitation_sum=np.zeros([self.n_en(self.neuron_thetas))
        excitation_sum=direction_neuron_config.get_blank_neurons()
        distance_excitation_sum=distance_neuron_config.get_blank_neurons()
        weight_sum=0
        distance_weight_sum=0
        for track in tracks:
            weight=self.get_track_attention(track,image_timestamp)
            if weight==0:
                continue
            angle=self.track_center_to_angle(track["center"],image_timestamp)
            logger.debug("image angle is {}".format(angle))
            pitch=self.track_center_to_pitch(track["center"],image_timestamp)
            sigma=(6.0/360)*(2*np.pi)
            sigma_pitch=(10.0/360)*(2*np.pi)
            #excitation_sum+=self.get_excited_direction_with_pitch(angle,sigma,pitch,sigma_pitch)
            excitation_sum+=direction_neuron_config.get_excitement(angle,sigma,pitch,sigma_pitch)
            #for i in range(self.n_pitch_neurons):
            #    excitation_sum[:,i]+=weight*self.get_excited_direction(angle,sigma)
            weight_sum+=weight
            if "spatial_array" in track and track["spatial_array"][2]!=0:
                dist=np.linalg.norm(track["spatial_array"])
                #logger.debug("distance is {}".format(dist))
                #distance_excitation_sum+=distance_neuron_config.get_excitement(dist,max(0.2*dist,0.1))
                my_sigma=abs(distance_neuron_config.neuron_distances[1]-distance_neuron_config.neuron_distances[0])
                distance_excitation_sum+=distance_neuron_config.get_excitement(dist,my_sigma)
                distance_weight_sum+=weight

        if weight_sum==0:
            return
        if weight_sum>1:
            self.last_attention_in_fov=time.time()
            excitation_sum=excitation_sum/weight_sum
            weight_sum=1
            if distance_weight_sum>0:
                distance_excitation_sum=distance_excitation_sum/distance_weight_sum
                #logger.debug("distance excitation sum {}".format(distance_excitation_sum))
                distance_weight_sum=1

        #self.direction_neurons=self.direction_neurons*(1-weight_sum)+excitation_sum
        #self.direction_neurons=np.clip(self.direction_neurons+excitation_sum,-1,1)
        #self.distance_neurons=np.clip(self.distance_neurons+distance_excitation_sum,-1,1)
        self.direction_neurons=self.direction_neurons+excitation_sum
        self.distance_neurons=self.distance_neurons+distance_excitation_sum
        self.rescale()
        self.emit_heading_message()
        #for i in range(self.n_pitch_neurons):
        #    self.direction_neurons[:,i]=np.clip(self.direction_neurons[:,i]+excitation_sum,-1,1)



    def draw_object_map(self):

        #downsampled=np.clip(resample(self.direction_neurons,12),-1,1)
        ####   For ring leds
        downsampled=np.zeros(12)
        conversion=self.direction_neurons.shape[0]/12
        flattened=self.direction_neurons.max(axis=1)
        for i in range(12):
            center=int(i*conversion)
            vals=np.take(flattened,[center-1,center,center+1],mode='wrap')
            downsampled[i]=(vals[0]+2*vals[1]+vals[2])/4
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

        #for window
#        xsize=320
#        ysize=240
#        toshow=np.zeros([ysize,xsize,3],np.uint8)
#        #cv.arrowedLine(toshow,(xsize//2,ysize//2),(xsize//2,ysize//2),(255,255,255),1, cv.LINE_AA, 0, 0.3)
#        cv.arrowedLine(toshow,(xsize//2,ysize//2),(xsize//2+20,ysize//2),(255,255,255),1, cv.LINE_AA, 0, 0.3)
#        for i in range(self.n_pitch_neurons):
#            self.draw_ring(toshow,self.direction_neurons[:,i],55+15*i)
#        #intensities=128+128*np.clip(self.direction_neurons[:,0],-1,1)
#        #for i in range(self.n_direction_neurons):
#        #    theta=self.neuron_thetas[i]
#        #    x=int(xsize//2+step*np.cos(theta))
#        #    y=int(ysize//2+step*np.sin(theta))
#        #    c=int(intensities[i])
#        #    #logger.debug(c)
#        #    cv.circle(toshow,(x,y),3,(c,c,c),4)
#        self.display.update_image("peripersonalspace",toshow)
