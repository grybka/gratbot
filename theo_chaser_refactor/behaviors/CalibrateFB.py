from behaviors.GratbotBehavior import GratbotBehavior
from behaviors.GratbotBehavior import GratbotBehavior_Loop
from behaviors.GratbotBehavior import GratbotBehavior_Wait
from behaviors.GratbotBehavior import GratbotBehaviorStatus
from behaviors.GratbotBehavior import GratbotBehavior_Series
from behaviors.GratbotBehavior import GratbotBehavior_RecordSensorToMemory
from behaviors.GratbotBehavior import GratbotBehavior_CopyMemory
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random
import json
import sys,os,traceback

import logging

class IsUltrasonic(GratbotBehavior):
    def __init__(self,greaterthan_not_lessthan,value):
        self.greaterthan_not_lessthan=greaterthan_not_lessthan
        self.value=value

    def act(self,comms,sensors):
        mycase=(sensors.gratbot_state["ultrasonic_sensor/last_measurement"]["average_distance"]>self.value)
        print("distance {} vs {}".format(sensors.gratbot_state["ultrasonic_sensor/last_measurement"]["average_distance"],self.value))
        if self.greaterthan_not_lessthan:
            mycase = not mycase
        if mycase:
            print("Returning Complete")
            return GratbotBehaviorStatus.COMPLETED
        else:
            print("Returning Failed")
            return GratbotBehaviorStatus.FAILED

class FBRandomAmount(GratbotBehavior):
    def __init__(self,forward_magnitude=0.5,back_magnitude=-0.5):
        super().__init__()
        self.forward_magnitude=forward_magnitude
        self.back_magnitude=back_magnitude
        self.speed=0.6

    def act(self,comms,sensors):
        mag=random.uniform(self.back_magnitude,self.forward_magnitude)
        print("fb between {} and {} I get {}".format(self.back_magnitude,self.forward_magnitude,mag))
        translation=[self.speed*np.sign(mag),0,0,np.abs(mag)/self.speed]
        sensors.short_term_memory["last_translation"]=translation
        #comms.set( ["drive","translate"],sensors.interpret_translation(translation))
        print("moving {}".format(translation))
        sensors.send_command(comms, ["drive","translate"],translation)
        #sensors.send_command(comms, ["drive","translate"],sensors.interpret_translation(translation))
        return GratbotBehaviorStatus.COMPLETED

class CalibrateFBToDistance(GratbotBehavior):
    def __init__(self):
        super().__init__()
        self.loop=None
        self.delta_dists=[]
        self.fb_magnitudes=[]

    def setup_loop(self,sensors):
        d=sensors.gratbot_state["ultrasonic_sensor/last_measurement"]["average_distance"]
        if d<0.5:
            fb=FBRandomAmount(left_magnitude=-1.0,right_magnitude=0.0)
        elif d>1.5:
            fb=FBRandomAmount(left_magnitude=0.0,right_magnitude=1.0)
        else:
            fb=FBRandomAmount(left_magnitude=-1.0,right_magnitude=1.0)

        self.loop=GratbotBehavior_Series([
            GratbotBehavior_RecordSensorToMemory("ultrasonic_sensor/last_measurement","start_distance"),
            fb,
            GratbotBehavior_Wait(0.6),
            GratbotBehavior_RecordSensorToMemory("ultrasonic_sensor/last_measurement","end_distance")])
        return True


    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="FB Magnitude",ylabel="Distance")
        ax.grid()
        plt.show()

    def act(self,comms,sensors):
        if self.loop==None:
            sensors.save_updates=True
            self.setup_loop(sensors)
            return GratbotBehaviorStatus.INPROGRESS
        ret=self.loop.act(comms,sensors)
        if ret==GratbotBehaviorStatus.FAILED:
            self.loop=None
            print("failed, restarting")
            return GratbotBehaviorStatus.FAILED
        if ret==GratbotBehaviorStatus.COMPLETED:
            #note the swapped minus signs because it's distance to the wall
            delta_dist=-sensors.short_term_memory["end_distance"]["average_distance"]+sensors.short_term_memory["start_distance"]["average_distance"]
            print("delta dist {}".format(delta_dist))
            self.delta_dists.append(delta_dist)
            print("translation magnitude {}".format(sensors.short_term_memory["last_translation"][0]))
            self.fb_magnitudes.append(sensors.short_term_memory["last_translation"][0])
            #fit
            print("number of elems {}".format(len(self.fb_magnitudes)))
            if len(self.fb_magnitudes)>30:
                sensors.fb_predictor.fit_calibration(self.fb_magnitudes,self.delta_dists)
                return GratbotBehaviorStatus.COMPLETED
            self.loop.reset()
            #need more
        return GratbotBehaviorStatus.INPROGRESS
