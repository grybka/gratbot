from Behavior import *
from Behavior import GratbotBehaviorStatus
from MotionBehaviors import *
import time
import numpy as np
from math import sin,cos
from underpinnings.BayesianArray import BayesianArray

#behaviors focused on exploring the map

#rough outline

#enumerate movement options
#for each movement option, predict amount of new information it would goin
#do the one with the moste new information
#finishes....  when?


class ExploreBehavior(GratbotBehavior):
    def __init__(self,gridmap=None):
        #pointer to MotionEstimationGyrus
        #self.motionestimationgyrus=motionestimationgyrus

        #OccupancyMap
        self.gridmap=gridmap
        self.sub_behavior=None

    def act(self,**kwargs):
        #if we have a sub behavior, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
        else:
            step_status=GratbotBehaviorStatus.COMPLETED
        broker=kwargs["broker"]
        if step_status==GratbotBehaviorStatus.COMPLETED:
            #if the sub behavior is finished, then figure out what to do next
            self.gridmap=kwargs["local_map"]
            if "latest_pose" not in kwargs["short_term_memory"]:
                #in case its loading
                return GratbotBehaviorStatus.INPROGRESS
            #print(kwargs["short_term_memory"]["latest_pose"])

            pose=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose"]["latest_pose"])
            move,info=self.do_thing(pose)
            if info<10:
                return GratbotBehaviorStatus.COMPLETED
            print("planning move {} with expected info {}".format(move,info))
            if move[0]=="turn":
                self.sub_behavior=GratbotBehavior_Series([Turn(move[1]),GratbotBehavior_Wait(2.0)])
            if move[0]=="ahead":
                self.sub_behavior=GratbotBehavior_Series([MoveAhead(move[1]),GratbotBehavior_Wait(2.0)])
            step_status=GratbotBehaviorStatus.INPROGRESS
        return step_status

    def predict_info(self,pose):
        #some stuff that shouldn't be hardcoded
        ultrasonic_maxrange=1.5
        lidar_maxrange=3.0
        ultrasonic_halfcone=7*2*np.pi/360
        lidar_halfcone=0.6 #(about 35 degrees)
        ultrasonic_info_weight=0.10 #because honestly it isn't as good

        p=pose
        #p is now a prediction of the pose after motion
        range_guess=self.gridmap.guess_range_to_wall(p)
        #here is my guess about info from the ultrasound
        loginfo_ultrasonic=self.gridmap.delta_logsum_explored_in_cone(p,min(ultrasonic_maxrange,range_guess),-ultrasonic_halfcone,ultrasonic_halfcone)
        #here is my guess about info from the vidar
        loginfo_vidar=self.gridmap.delta_logsum_explored_in_cone(p,min(lidar_maxrange,range_guess),-lidar_halfcone,lidar_halfcone)
        total_info=(ultrasonic_info_weight*loginfo_ultrasonic+loginfo_vidar)
        return total_info

    def do_thing(self,pose):
        #enumerate movement options
        #let's hard code those for now
        #each of these implies a lidar and ultrasonic scan afterward
        movement_options=[['turn',15*2*np.pi/360],
                          ['turn',45*2*np.pi/360],
                          ['turn',-15*2*np.pi/360],
                          ['turn',-45*2*np.pi/360],
                          ['ahead',0.2],
                          ['ahead',-0.2]]


        #for each movement option, predict amount of new information it would goin
        info_predictions=[]
        for i in range(len(movement_options)):
            m=movement_options[i]
            if m[0]=='turn':
                #predict new pose
                p=pose.copy()
                p.vals+=np.array([0,0,m[1]])
                info_predictions.append(self.predict_info(p))
            if m[0]=='ahead':
                #predict new pose
                p=pose.copy()
                p.vals+=np.array([m[1]*sin(p.vals[2]),m[1]*cos(p.vals[2]),0])
                info_predictions.append(self.predict_info(p))
            print("A {} of {} should give info of {}".format(m[0],m[1],info_predictions[i]))
        best_i=np.argmax(info_predictions)
        return movement_options[best_i],info_predictions[best_i]
