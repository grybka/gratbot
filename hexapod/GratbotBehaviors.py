# Behaviors for hexapod

import numpy as np
import time
import filterpy
#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise


class GratbotBehavior:
    def __init__(self, comms):
        self.comms = comms
    def act(self,video_objects):
        return

class WaveAtFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.waving=False
    def act(self,video_objects):
        wave_controller="wave_paw"
        idle_controller="legs_idle"
        if self.waving==False and "faces" in video_objects:
            self.waving=True
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 0 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"left_speed","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"right_speed","SET" ], 1 )
        elif self.waving==True and "faces" not in video_objects:
            self.waving=False
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 0 )


class HeadTrackFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
#        self.filter_y=KalmanFilter(dim_x=2,dim_z=1)
#        self.filter_y.x=np.array([2.,0.]) #pos, vel
#        self.filter_y.F=np.array([[1.0,1.0],[0.,1.]]) #x=x0+vt
#        self.filter_y.P *= 1000. #covariance, not sue
#        self.filter_y.H =np.array([ [1.0,0]])
#        self.filter_y.R = np.array([[10]]) #measurement noise
#        self.filter_y.Q=Q_discrete_white_noise(dim=2,dt=0.1,var=0.13)
#
#        self.filter_x=KalmanFilter(dim_x=2,dim_z=1)
#        self.filter_x.x=np.array([2.,0.]) #pos, vel
#        self.filter_x.F=np.array([[1.0,1.0],[0.,1.]]) #x=x0+vt
#        self.filter_x.P *= 1000. #covariance, not sue
#        self.filter_x.H =np.array([ [1.0,0]])
#        self.filter_x.R = np.array([[10]]) #measurement noise
#        self.filter_x.Q=Q_discrete_white_noise(dim=2,dt=0.1,var=0.13)
        self.last_time=time.time()
    def act(self, video_objects):
        now=time.time()
        act_often=0.1
        if now-self.last_time < act_often:
            return
        self.last_time=now
        video_height=480
        video_width=640
        pixel_to_servo=0.1
        if "faces" in video_objects:
            face=video_objects["faces"][0]
            centerx=0.5*(face["startx"]+face["endx"])
            centery=0.5*(face["starty"]+face["endy"])
            print("face at {}, {}".format(centerx,centery))
            dx=video_width/2-centerx #how does it know the size
            dy=video_height/2-centery
#            self.filter_y.predict()
#            self.filter_x.predict()
#            self.filter_y.update([dy])
#            self.filter_x.update([dx])
#            dy=self.filter_y.x[0]
#            dx=self.filter_x.x[0]
            print("dy updated {}".format(dy))
            dx_servo=dx*pixel_to_servo
            dy_servo=dy*pixel_to_servo
            #print("move {},{}".format(dx_servo,dy_servo))
            #print("move {}".format(dy_servo))
            self.comms.set_intention( ["camera_x","position_delta","SET" ], dx_servo )
            self.comms.set_intention( ["camera_y","position_delta","SET" ], dy_servo )
        else:
            print("no face")
            #self.filter_y.predict()
            #self.comms.set_intention( ["camera_x","position","SET" ], 0 )
            #self.comms.set_intention( ["camera_y","position","SET" ], 0 )
