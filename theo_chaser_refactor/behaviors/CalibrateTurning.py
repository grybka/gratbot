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

class TurnRandomAmount(GratbotBehavior):
    def __init__(self,left_magnitude=-1,right_magnitude=1):
        super().__init__()
        self.left_magnitude=left_magnitude
        self.right_magnitude=right_magnitude

    def act(self,comms,sensors):
        translation=[0,0,random.uniform(self.left_magnitude,self.right_magnitude)]
        sensors.short_term_memory["last_translation"]=translation
        #comms.set( ["drive","translate"],sensors.interpret_translation(translation))
        sensors.send_command(comms, ["drive","translate"],sensors.interpret_translation(translation))
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

class ObjectTrackToMemory(GratbotBehavior):
    def __init__(self,object_id):
        super().__init__()
        self.object_id=object_id

    def act(self,comms,sensors):
        try:
            #x,y,w,h=sensors.tracker.retrieve_object(self.object_id).get_predictions()
            x,y,w,h=sensors.tracker.retrieve_object(self.object_id).last_update
            sensors.update_memory("tracked_object_x",x)
            sensors.update_memory("tracked_object_y",y)
            sensors.update_memory("tracked_object_w",w)
            sensors.update_memory("tracked_object_h",h)
        except Exception as e:
            print("Tracking Probably lost")
            #print("Exception: {}".format(e))
            #exc_type, exc_obj, exc_tb = sys.exc_info()
            #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            #print(exc_type, fname, exc_tb.tb_lineno)
            #traceback.print_exc(file=sys.stdout)
            return GratbotBehaviorStatus.FAILED
        return GratbotBehaviorStatus.COMPLETED

class CalibrateTurnToVideo(GratbotBehavior):
    def __init__(self):
        super().__init__()
        self.turn_magnitudes=[]
        self.start_angles=[]
        self.end_angles=[]
        self.delta_angles=[]
        self.loop=None
        self.target_object=None
        self.delta_vision=[]
        self.start_vision=[]
        self.stop_vision=[]
        self.counter=0

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

    def setup_loop(self,sensors):
        self.counter+=1
        #TODO should I select only certain objects to track?
        if len(sensors.tracker.tracked_objects)>0:
            track_id=sensors.tracker.tracked_objects[0].id
            print("tracking {}".format(track_id))
        else:
            print("no tracked objects")
            return False
        #x,y,w,h=sensors.tracker.retrieve_object(track_id).get_predictions()
        x,y,w,h=sensors.tracker.retrieve_object(track_id).last_update
        print(x)
        if x<0:
           myturn= TurnRandomAmount(left_magnitude=0,right_magnitude=1.0)
        else:
            myturn= TurnRandomAmount(left_magnitude=-1.0,right_magnitude=0.0)
        self.loop=GratbotBehavior_Series([
            ObjectTrackToMemory(track_id),
            GratbotBehavior_RecordSensorToMemory("compass_heading","start_compass_heading"),
            GratbotBehavior_CopyMemory("tracked_object_x","start_x"),
            GratbotBehavior_CopyMemory("tracked_object_y","start_y"),
            myturn,
            GratbotBehavior_Wait(0.75),
            ObjectTrackToMemory(track_id),
            GratbotBehavior_RecordSensorToMemory("compass_heading","end_compass_heading"),
            GratbotBehavior_CopyMemory("tracked_object_x","stop_x"),
            GratbotBehavior_CopyMemory("tracked_object_y","stop_y")])
        return True

    def act(self,comms,sensors):
        if self.loop==None:
            sensors.save_updates=True
            self.setup_loop(sensors)
            return GratbotBehaviorStatus.INPROGRESS
        ret=self.loop.act(comms,sensors)
        if ret==GratbotBehaviorStatus.FAILED:
            self.loop=None
            print("failed, restarting")
            return GratbotBehaviorStatus.INPROGRESS
        if ret==GratbotBehaviorStatus.COMPLETED:
            #-----------compas heading------
            delta_heading=sensors.short_term_memory["end_compass_heading"]-sensors.short_term_memory["start_compass_heading"]
            self.start_angles.append(sensors.short_term_memory["start_compass_heading"])
            self.end_angles.append(sensors.short_term_memory["end_compass_heading"])
            while delta_heading>np.pi:
                delta_heading=delta_heading-2*np.pi
            while delta_heading<-np.pi:
                delta_heading=delta_heading+2*np.pi
            self.delta_angles.append(delta_heading)
            #-----------Tracking
            self.delta_vision.append(
sensors.short_term_memory["stop_x"]-sensors.short_term_memory["start_x"])

            #motion
            self.turn_magnitudes.append(sensors.short_term_memory["last_translation"][2])
            #fit
            if len(self.turn_magnitudes)>20:
                sensors.turn_predictor.fit_calibration(self.turn_magnitudes,self.delta_vision,self.delta_angles)
            #print("number of elems {}".format(len(self.turn_magnitudes)))
            #    [m,b]=np.polyfit(self.turn_magnitudes,self.delta_angles,1)
#
#                self.show_plot(self.turn_magnitudes,self.delta_angles,m,b)
#                [m2,b2]=np.polyfit(self.turn_magnitudes,self.delta_vision,1)
#                self.show_plot(self.turn_magnitudes,self.delta_vision,m2,b2)
#                [m3,b3]=np.polyfit(self.delta_vision,self.delta_angles,1)
#                self.show_plot(self.delta_vision,self.delta_angles,m3,b3)
                #self.show_plot(np.arange(len(self.turn_magnitudes)),self.start_angles,self.end_angles,self.delta_angles)
                return GratbotBehaviorStatus.COMPLETED
            self.loop=None
            #need more
        return GratbotBehaviorStatus.INPROGRESS
