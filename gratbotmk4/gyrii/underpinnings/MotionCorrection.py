from collections import deque
import numpy as np


def get_closest_value(timestamp,mylist):
        #for mylist ordered by [time,value], return the value that is closest to the input timestamp
        if len(mylist)==0:
            return None
        if timestamp<=mylist[0][0]:
            return mylist[0][1]

        first_val=mylist[0]
        second_val=None
        for val in mylist:
            if timestamp>val[0]:
                second_val=val
            else:
                first_val=val
        if abs(first_val[0]-timestamp)<abs(second_val[0]-timestamp):
            return first_val[1]
        else:
            return second_val[1]
class MotionCorrection: #to correct image frames from heading changes
    def __init__(self,max_recent_history=20):
        self.max_recent_history=max_recent_history
        self.gyros=deque([],maxlen=self.max_recent_history)
        self.accel=np.array([0,0,10]) #for gravity
        self.headings=deque([],maxlen=self.max_recent_history) #from gyro integration
        #self.last_used_heading=0
        self.initialized=False
        self.last_used_heading=np.array([0,0,0])
        self.angle_heading_slope=-1.515
        self.angle_ygyro_slope=-1.6
        self.z_gyro_index=0
        self.y_gyro_index=2
        self.x_gyro_index=1

    def read_message(self,message):
        if 'packets' in message: #rotation, etc
            if len(self.headings)==0: #for the first packet received
                #self.headings.append([message['packets'][0]["gyroscope_timestamp"],0 ])
                self.headings.append([message['packets'][0]["gyroscope_timestamp"],np.array([0,0,0]) ])
            for packet in message["packets"]:
                #self.gyros.append( [packet["gyroscope_timestamp"],packet["gyroscope"]])
                self.accel=0.8*self.accel+0.2*np.array(packet["acceleration"])
                    #TODO I could do a fancier integration
                #next_heading=self.headings[-1][1]+np.array(packet["gyroscope"])*(packet["gyroscope_timestamp"]-self.headings[-1][0])
                next_heading=np.array(packet["local_rotation"])
                self.headings.append( [packet["gyroscope_timestamp"],next_heading])

    def get_offset_and_update(self,image_timestamp):
        if len(self.headings)==0:
            return 0,0
        closest_heading_vec=get_closest_value(image_timestamp,self.headings)
        delta_heading=closest_heading_vec-self.last_used_heading
        self.last_used_heading=closest_heading_vec #this is the update part
        if self.initialized==False:
            self.initialized=True
            return 0,0
        #offset=delta_heading*self.angle_heading_slope
        #return offset
        #offset_x=delta_heading[self.z_gyro_index]*self.angle_heading_slope
        #TODO figure out how to line this up with gravity
        mag_accel=np.linalg.norm(self.accel)
        cos_angle=self.accel[1]/mag_accel
        sin_angle=self.accel[2]/mag_accel
        turn_mag=delta_heading[self.z_gyro_index]*cos_angle-delta_heading[self.y_gyro_index]*sin_angle
        #offset_x=delta_heading[self.z_gyro_index]*self.angle_heading_slope
        offset_x=turn_mag*self.angle_heading_slope
        offset_y=delta_heading[self.x_gyro_index]*self.angle_ygyro_slope
        return offset_x,offset_y
