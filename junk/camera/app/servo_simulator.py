import math
import numpy as np
import torch
import torch.nn as nn

class ServoSimulator:
    def __init__(self):
        self.degrees_per_step=1
        self.no_motion_prob=0.1
        self.degree_uncertainty=2
        self.max_degrees=400
        self.min_degrees=-400

    def apply_motion(self,desired_steps,old_position):
#print("old positoin {}".format(old_position))
        new_position=old_position
        if np.random.random()>self.no_motion_prob:
            new_position+=desired_steps*self.degrees_per_step+self.degree_uncertainty*np.random.normal()
#print("new pos {}".format(new_position))
        if new_position>self.max_degrees:
            new_position=self.max_degrees
        if new_position<self.min_degrees:
            new_position=self.min_degrees
        return new_position

class PIDControl:
    def __init__(self):
        self.integral_length=5

    def data_array_to_p_i_d(self,data_array):
        #presume data_array is a numpy array of the error signal
        P=data_array[-1]
        I=0
        if len(data_array)<self.integral_length:
            I=np.sum(data_array)/len(data_array)
        else:
            I=np.sum(data_array[-self.integral_length:])/self.integral_length
        D=0
        if len(data_array)>1:
            D=data_array[-1]-data_array[-2]
        return P,I,D

    def calculate_control(self,data_array):
        P,I,D=self.data_array_to_p_i_d(data_array)
        return self.calculate_pid_control(P,I,D)

    def calculate_pid_control(self,p,i,d):
        raise Exception("calculate pid control not implemented")

    def train(data_array,control_signals):
        pass


class PIDControlFixed(PIDControl):
    def __init__(self):
        self.integral_length=5
        self.P=1.94
        self.I=0.1
        self.D=0

    def calculate_pid_control(self,p,i,d):
        return self.P*p+self.I*i+self.D*d

def clamp_max(x,mx):
    if x>mx:
        return mx
    if x<-mx:
        return -mx
    return x


class PIDControlTrainable(PIDControl):
    def __init__(self):
        self.integral_length=5
#self.P=2.0
        self.P=1.0
        self.I=0.1
        self.D=0
        self.training_rate=0.01

    def calculate_pid_control(self,p,i,d):
        return self.P*p+self.I*i+self.D*d


    def train(self,data_array):
#        p_array=np.zeros(len(data_array))
#        i_array=np.zeros(len(data_array))
#        d_array=np.zeros(len(data_array))
        dPsum=0
        dIsum=0
        dDsum=0
        for i in range(1,len(data_array)-1):
            myp,myi,myd=self.data_array_to_p_i_d(data_array[0:i])
#            p_array[i]=p
#            i_array[i]=i
#            d_array[i]=d
            actual_vec=data_array[i+1]-data_array[i]
            target_vec=0-data_array[i]
            desired_change=target_vec-actual_vec
            #print("trying to change by {}".format(target_vec))
            #print("actually changed by {}".format(actual_vec))
            #output=myp*P+...
            #doutput/dP=myp
            dP=desired_change*myp
            dI=desired_change*myi
            dD=desired_change*myd
        self.P-=clamp_max(dPsum,self.training_rate)
        self.I-=clamp_max(dIsum,self.training_rate)
        self.D-=clamp_max(dDsum,self.training_rate)

#---Neural Network Here

class ControlsPredictor(nn.Module):
    #one dimensional, for experimentation

    def __init__(self):
        super(ControlsPredictor,self).__init__() 
        self.past_size=3
        self.middle=nn.Linear(self.past_size+1,1)

    def forward(self,x):
        x=self.middle(x)
        return x

    def predict(self,input_array,control_signal):
        #given an array of previous measurements and a proposed control signal
        #predict what the next measurement will be

        #TODO crash if past size too small
        x=torch.zeros(self.past_size+1)
        x[0]=control_signal
        for i in range(self.past_size):
            x[i+1]=input_array[-(i+1)]
        return self.forward(x)

    def train(self,input_arrays,control_signals):
        #build training vectors
#training_vectors=torch.zeros(self.past_size+1,len(input_arrays)-self.past_size-1)
        training_vectors=torch.zeros(len(input_arrays)-self.past_size-1,self.past_size+1)
#target_vectors=torch.zeros(1,len(input_arrays)-self.past_size-1)
        target_vectors=torch.zeros(len(input_arrays)-self.past_size-1,1)
        for i in range(len(input_arrays)-self.past_size-1):
            training_vectors[i][0]=control_signals[i+self.past_size-1]
            for j in range(self.past_size):
                training_vectors[i][j+1]=input_arrays[i+self.past_size-j-1]
            target_vectors[i][0]=input_arrays[i+self.past_size+1]
#print("input_array: {}".format(input_arrays))
#print("control_signals: {}".format(control_signals))
#print("training_vectors: {}".format(training_vectors))
#print("target: {}".format(target_vectors))
        #set up the loss
        loss_fn=nn.MSELoss()
#loss_fn=nn.SmoothL1Loss()
#optimizer = torch.optim.SGD(self.parameters(), lr=1e-4)
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-2)
        for t in range(100):
            #zero gradients
            optimizer.zero_grad()
            #forward backward optimize
            y_pred=self.forward(training_vectors)
            loss=loss_fn(y_pred,target_vectors)
            loss.backward()
            optimizer.step()
#print("#loss: {}".format(loss.item()))

        
def find_best_control_signal(predictor,input_array,target_output):
    guess=0
    eps=0.001
    print("#I'm at {}".format(input_array[-1]))
    print("#I want to go to {}".format(target_output))
    for i in range(10):
        y=((predictor.predict(input_array,guess)-target_output)**2).item()
#print("y is {}".format(y))
        yp=((predictor.predict(input_array,guess+eps)-target_output)**2).item()
        grad=(yp-y)/eps
        if grad==0:
            break
        newguess=guess-y/grad
#print("{}".format(y))
#print("guess y grad newguess")
#print("{} {} {}".format(guess,predictor.predict(input_array,guess).item(),target_output))
        guess=newguess
#print("******")
    return guess



#-----test here-----

sim=ServoSimulator()


#generate target dataset
test_size=1000
test_flip=20
max_signal=50
target_pos=np.zeros(test_size)
pos=0
initial_pos=0
while pos+test_flip<test_size:
    for i in range(pos,pos+test_flip):
        target_pos[i]=90
    pos=pos+2*test_flip
#for i in range(test_size):
#    target_pos[i]=test_flip*math.sin(i/100)
    
controls=PIDControlFixed()
#controls=PIDControlTrainable()
predictor=ControlsPredictor()

#create controls
#controls=NNServoControl()
#controls=OneParamServoControl()
#controls=BayseanServoControl()

#TODO exercise controls
pos_record=[]
desire_record=[]

constant_record=np.zeros(len(target_pos))
sigma_record=np.zeros(len(target_pos))
prediction_record=np.zeros(len(target_pos))
pos_record.append(initial_pos)
for i in range(1,len(target_pos)):
    if i<=3:
        desired_motion=0
    elif i<3:
#desired_motion=controls.calculate_control_signal(pos_record[i-3:i],target_pos[i])
        errors=target_pos[i]*np.ones(3)-pos_record[-3:]
        desired_motion=controls.calculate_control(errors)
        prediction_record[i]=predictor.predict(pos_record,desired_motion)
#print("signals {} {}".format(pos_record[i-1],target_pos[i]))
#desired_motion=controls.calculate_control_signal(pos_record[i-1],target_pos[i])
    else:
        desired_motion=find_best_control_signal(predictor,pos_record,target_pos[i])
        desired_motion=clamp_max(desired_motion,max_signal)
    desire_record.append(desired_motion)
    pos_record.append(sim.apply_motion(desired_motion,pos_record[-1]))
    count=20
    if i>0 and i%count==0:
        predictor.train(pos_record[-2*count:],desire_record[-2*count:])
#        controls.train(pos_record[i-10:i])
#print("first 10 is {}".format(pos_record[0:10]))
#print("i {} i-1 {}".format(i,i-1))
#print("pos record i {} {} dm {}".format(pos_record[i-1],pos_record[i],desired_motion))
#print("future {}".format(pos_record[i+1]))
#if i>4:
#controls.train_on_event(pos_record[i-1],pos_record[i],target_pos[i])
#controls.train_on_event(pos_record[i-3:i],pos_record[i],target_pos[i])
#constant_record[i]=controls.proportionality
#sigma_record[i]=controls.sigma_proportionality



#Print out result
#print("#element target_pos actual_pos")
for i in range(1,len(target_pos)):
    print("{} {} {} {} {}".format(i,target_pos[i],pos_record[i],prediction_record[i],prediction_record[i]-pos_record[i-1]))

#print out the shape
#for i in range(-40,40):
#    p=predictor.predict([0,0,0],i)
#    print("{} {}".format(i,p.item()))


"""

class BayseanServoControl:
    def __init__(self):
        self.proportionality=0.1
        self.sigma_proportionality=10
        self.sigma_measurement=10.0
        self.max_sigma_for_update=10
        self.min_sigma=1.0

    def calculate_control_signal(self,current_position,target_position):
        return (current_position-target_position)*self.proportionality

    def train_on_event(self,old_position,new_position,target_position):
        my_signal=self.calculate_control_signal(old_position,target_position)
        new_prop=my_signal/(old_position-new_position)
        sigma_newprop=abs(self.sigma_measurement*my_signal/(old_position-new_position)**2)
#print("my signal {}".format(my_signal))
#print("old, new, target {},{},{}".format(old_position,new_position,target_position))
#print("oldprop {} newprop {}".format(self.proportionality,new_prop))
#print("sigma_oldprop {} sigma_newprop {}".format(self.sigma_proportionality,sigma_newprop))


        if sigma_newprop>self.max_sigma_for_update:
            return
        if sigma_newprop<self.min_sigma:
            sigma_newprop=self.min_sigma
        if self.sigma_proportionality<self.min_sigma:
            self.sigma_proportionality=self.min_sigma
        isig1=1/self.sigma_proportionality**2
        isig2=1/sigma_newprop**2

        self.proportionality=(self.proportionality*isig1+new_prop*isig2)/(isig1+isig2)
        self.sigma_proportionality=1/np.sqrt(isig1+isig2)
    

        


class OneParamServoControl:
    def __init__(self):
        self.proportionality=0.1
        self.max_training_constant=0.1
        self.expected_noise=10.0

    def calculate_control_signal(self,current_position,target_position):
        return (current_position-target_position)*self.proportionality

    def train_on_event(self,old_position,new_position,target_position):
        my_signal=self.calculate_control_signal(old_position,target_position)
        snr=abs(old_position-new_position)/self.expected_noise
        if snr==0: 
            return #no training with no snr
        if snr>self.max_training_constant:
            snr=self.max_training_constant
#print("training: {} {} {} {}".format(old_position,new_position,target_position,my_signal))
        new_prop=my_signal/(old_position-new_position)
        self.proportionality=self.proportionality+(new_prop-self.proportionality)*snr

class NNServoControl(nn.Module):
    def __init__(self):
        super(NNServoControl,self).__init__()
        #lets assume there are 3 past inputs used
        #linear control part
        #self.matrix_part = torch.ones(3, 1, requires_grad=True)
        self.matrix_part = torch.rand(1, 3, requires_grad=True)+torch.ones(1,3,requires_grad=True)
#self.matrix_part = torch.rand(1, 1, requires_grad=True)
        self.training_parameter=0.01
        self.clamp_delta=30.0
        self.clamp_train=1.0

    def forward(self,x):
        return torch.matmul(self.matrix_part,x)


    def calculate_control_signal(self,input_array,target_position):
#input array here is an array of errors between actual position and target position
        input_array_torch=torch.zeros(len(input_array))
        for i in range(len(input_array_torch)):
            input_array_torch[i]=input_array[i]
        input_array=input_array_torch
        
        delta_array=target_position*torch.ones(len(input_array))-input_array
        return self.forward(delta_array)

    def train_on_event(self,input_array,position_after,target_position):
        my_out=self.calculate_control_signal(input_array,target_position)
        input_array_torch=torch.zeros(len(input_array))
        for i in range(len(input_array_torch)):
            input_array_torch[i]=input_array[i]
        input_array=input_array_torch
        

        self.zero_grad()

        old_errs=(target_position*torch.ones(len(input_array))-input_array).detach()
        new_err=target_position-position_after
        actual_motion=(new_err-old_errs[-1])

        my_out.backward()
#print("my_out {}".format(my_out))
#print("diff {}".format(diff))
        for f in self.parameters():
            dw=diff*f.grad.data*self.training_parameter
            if dw>self.clamp_train:
               dw=self.clamp_train
            if dw<-self.clamp_train:
               dw=-self.clamp_train
            f.data.sub_(dw)


"""
