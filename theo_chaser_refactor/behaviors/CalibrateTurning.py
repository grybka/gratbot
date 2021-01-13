from behaviors.GratbotBehavior import GratbotBehavior
from behaviors.GratbotBehavior import GratbotBehavior_Loop
from behaviors.GratbotBehavior import GratbotBehavior_Wait
from behaviors.GratbotBehavior import GratbotBehaviorStatus
from behaviors.GratbotBehavior import GratbotBehavior_Series
from behaviors.GratbotBehavior import GratbotBehavior_RecordSensorToMemory
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random

class TurnRandomAmount(GratbotBehavior):
    def __init__(self,left_magnitude=-1,right_magnitude=1):
        super().__init__()
        self.left_magnitude=left_magnitude
        self.right_magnitude=right_magnitude

    def act(self,comms,sensors):
        translation=[0,0,random.uniform(self.left_magnitude,self.right_magnitude)]
        sensors.short_term_memory["last_translation"]=translation
        comms.set( ["drive","translate"],sensors.interpret_translation(translation))
        return GratbotBehaviorStatus.COMPLETED

class CalibrateTurnToAngle(GratbotBehavior):
    def __init__(self):
        super().__init__()
        self.loop=GratbotBehavior_Series([
            GratbotBehavior_RecordSensorToMemory("compass_heading","start_compass_heading"),
            TurnRandomAmount(left_magnitude=-1.0,right_magnitude=1.0),
            GratbotBehavior_Wait(0.6),
            GratbotBehavior_RecordSensorToMemory("compass_heading","end_compass_heading")])
        self.turn_magnitudes=[]
        self.start_angles=[]
        self.end_angles=[]
        self.delta_angles=[]

    def show_plot(self,x,y,m,b):
        test_x=np.linspace(-1,1,100)
        test_y=test_x*m
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(test_x,test_y)
        ax.set(xlabel="Turn Magnitude",ylabel="Angle Change")
        ax.grid()
        plt.show()


    def fit_data(self,sensors,turn_magnitudes,delta_angles):
        pass

    def act(self,comms,sensors):
        ret=self.loop.act(comms,sensors)
        if ret==GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED
        if ret==GratbotBehaviorStatus.COMPLETED:
            delta_heading=sensors.short_term_memory["end_compass_heading"]-sensors.short_term_memory["start_compass_heading"]
            self.start_angles.append(sensors.short_term_memory["start_compass_heading"])
            self.end_angles.append(sensors.short_term_memory["end_compass_heading"])
            while delta_heading>np.pi:
                delta_heading=delta_heading-2*np.pi
            while delta_heading<-np.pi:
                delta_heading=delta_heading+2*np.pi
            #Handle pi wraparound!!!  TODO
            self.turn_magnitudes.append(sensors.short_term_memory["last_translation"][2])
            self.delta_angles.append(delta_heading)
            #fit
            print("number of elems {}".format(len(self.turn_magnitudes)))
            if len(self.turn_magnitudes)>30:
                [m,b]=np.polyfit(self.turn_magnitudes,self.delta_angles,1)

                self.show_plot(self.turn_magnitudes,self.delta_angles,m,b)
                #self.show_plot(np.arange(len(self.turn_magnitudes)),self.start_angles,self.end_angles,self.delta_angles)
                return GratbotBehaviorStatus.COMPLETED
            self.loop.reset()
            #need more
        return GratbotBehaviorStatus.INPROGRESS
