
from Gyrus import ThreadedGyrus
import numpy as np
import time
from gyrii.underpinnings.GratbotLogger import gprint

#Predict what my sensors are going to say in a low level way

#At each time step, take as inputs and predict the following

#Motors on/off
#Visual odometer offset
#*Accelerometer
#*Gyroscope
#Compass

#Take as inputs, but do not predict
#Motor command requests

class LowLevelSensorPredictor(ThreadedGyrus):
    def __init__(self,broker):
        #here is my input array
        counter=0
        self.motor_active_index_start=counter
        counter+=3
        self.video_offset_index_start=counter
        counter+=1
        self.b_field_index_start=counter
        counter+=3
        self.gyro_index_start=counter
        counter+=3
        self.accel_index_start=counter
        counter+=3
        #This is the break between predicted and inputs
        #sum, and counts for average
        self.current_time_bin_target_vector=np.zeros(counter)
        self.current_time_bin_target_vector_counts=np.zeros(counter)

        counter=0
        self.motor_command_index_start=counter
        counter+=4
        #sum, and counts for average
        self.current_time_bin_extra_input_vector=np.zeros(counter)
        self.current_time_bin_extra_input_vector_counts=np.zeros(counter)

        #for timing
        self.last_time_update=0
        self.time_bin_width=0.1

        #for records
        self.target_vector_record=[]
        self.input_vector_record=[]
        self.save_every=10
        self.last_save=0
        super().__init__(broker)

    def execute_prediction(self,timestamp):
        #TODO start with last prediction instead of zeros
        target=np.zeros(len(self.current_time_bin_target_vector))
        for i in range(len(self.current_time_bin_target_vector)):
            if self.current_time_bin_target_vector_counts[i]!=0:
                target[i]=self.current_time_bin_target_vector[i]/self.current_time_bin_target_vector_counts[i]
        input=np.zeros(len(self.current_time_bin_extra_input_vector))
        for i in range(len(self.current_time_bin_extra_input_vector)):
            if self.current_time_bin_extra_input_vector_counts[i]!=0:
                input[i]=self.current_time_bin_extra_input_vector[i]/self.current_time_bin_extra_input_vector_counts[i]
        self.target_vector_record.append(target)
        self.input_vector_record.append(np.concatenate([target,input]))
        #TODO make predictions
        #reset to zero
        self.current_time_bin_target_vector=np.zeros(len(self.current_time_bin_target_vector))
        self.current_time_bin_target_vector_counts=np.zeros(len(self.current_time_bin_target_vector))
        self.current_time_bin_extra_input_vector=np.zeros(len(self.current_time_bin_extra_input_vector))
        self.current_time_bin_extra_input_vector_counts=np.zeros(len(self.current_time_bin_extra_input_vector))

    def save_record(self):
        if len(self.input_vector_record)==0:
            return
        gprint("saving record")
        with open("llsp_records.txt",'wb') as f:
            #for i in range(len(self.input_vector_record)):
            #    f.write("{}\n".format(self.input_vector_record[i]))
            #    f.write("{}\n".format(self.target_vector_record[i]))
            np.save(f,np.array(self.input_vector_record))
            np.save(f,np.array(self.target_vector_record))

    def read_message(self,message):
        if "timestamp" in message:
            if message["timestamp"]>self.last_time_update+self.time_bin_width:
                if self.last_time_update!=0:
                    self.execute_prediction(message["timestamp"])
                self.last_time_update=message["timestamp"]+self.time_bin_width
            if message["timestamp"]>self.last_save+self.save_every:
                self.save_record()
                self.last_save=message["timestamp"]

        if "drive/motors_active" in message:
            #TODO this would be nice as a running average
            motors_active=message["drive/motors_active"][0:3]
            duration=message["drive/motors_active"][3]
            self.current_time_bin_target_vector[self.motor_active_index_start:self.motor_active_index_start+3]+=motors_active[0:3]
            self.current_time_bin_target_vector_counts[self.motor_active_index_start:self.motor_active_index_start+3]+=np.ones(3)
        if "position_sensor/acceleration" in message:
            self.current_time_bin_target_vector[self.accel_index_start:self.accel_index_start+3]+=np.clip(message["position_sensor/acceleration"][0:3],)
            self.current_time_bin_target_vector_counts[self.accel_index_start:self.accel_index_start+3]+=np.ones(3)
        if "position_sensor/gyro" in message:
            self.current_time_bin_target_vector[self.gyro_index_start:self.gyro_index_start+3]+=message["position_sensor/gyro"][0:3]
            self.current_time_bin_target_vector_counts[self.gyro_index_start:self.gyro_index_start+3]+=np.ones(3)

        if "position_sensor/b_field" in message:
            #TODO this would be nice as a running average
            self.current_time_bin_target_vector[self.b_field_index_start:self.b_field_index_start+3]+=message["position_sensor/b_field"][0:3]
            self.current_time_bin_target_vector_counts[self.b_field_index_start:self.b_field_index_start+3]+=np.ones(3)
        if "video_offset" in message:
            #TODO this would be nice as a running average
            self.current_time_bin_target_vector[self.video_offset_index_start]+=message["video_offset"][2] #x velocity
            self.current_time_bin_target_vector_counts[self.video_offset_index_start]+=1 #x velocity
        if "motor_command" in message:
            if message["motor_command"]["type"]=="translate":
                motor4vector=message["motor_command"]["vector"]
            elif message["motor_command"]["type"]=="turn":
                duration=message["motor_command"]["magnitude"]
                turn_speed=0.6
                motor4vector=[0,0,np.sign(duration)*turn_speed,abs(duration)]
            elif message["motor_command"]["type"]=="ahead":
                duration=message["motor_command"]["magnitude"]
                fb_speed=0.6
                motor4vector=[np.sign(duration)*fb_speed,0,0,abs(duration)]
            elif message["motor_command"]["type"]=="slew":
                duration=message["motor_command"]["magnitude"]
                fb_speed=0.6
                motor4vector=[0,np.sign(duration)*fb_speed,0,abs(duration)]
            self.current_time_bin_extra_input_vector[self.motor_command_index_start:self.motor_command_index_start+4]=motor4vector[0:4]
            self.current_time_bin_extra_input_vector_counts[self.motor_command_index_start:self.motor_command_index_start+4]=np.ones(4)

    def get_keys(self):
        return [ "motor_command","drive/motors_active","position_sensor/b_field","position_sensor/gyro","position_sensor/acceleration","video_offset" ]

    def get_name(self):
        return "LowLevelSensorPredictor"
