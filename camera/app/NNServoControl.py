#controls a servo, learning as it goes

import math
import numpy as np
import torch
import torch.nn as nn

class ControlsPredictor(nn.Module):
    #one dimensional, for experimentation

    def __init__(self):
        super(ControlsPredictor,self).__init__() 
        self.past_size=3
        self.middle=nn.Linear(self.past_size+1,1)
        self.training_steps=100
        self.optimize_loops=10

    def forward(self,x):
        x=self.middle(x)
        return x

    def predict(self,input_array,control_signal):
        #given an array of previous measurements and a proposed control signal
        #predict what the next measurement will be

        x=torch.zeros(self.past_size+1)
        x[0]=control_signal
        for i in range(self.past_size):
            if i>len(input_array)
                x[i+1]=0
            else:
                x[i+1]=input_array[-(i+1)]
        return self.forward(x)

    def train(self,input_arrays,control_signals):
        if len(input_arrays)-self.past_size-1<1:
            return #not enough training data
        #build training vectors
        training_vectors=torch.zeros(len(input_arrays)-self.past_size-1,self.past_size+1)
        target_vectors=torch.zeros(len(input_arrays)-self.past_size-1,1)
        for i in range(len(input_arrays)-self.past_size-1):
            training_vectors[i][0]=control_signals[i+self.past_size-1]
            for j in range(self.past_size):
                training_vectors[i][j+1]=input_arrays[i+self.past_size-j-1]
            target_vectors[i][0]=input_arrays[i+self.past_size+1]
        #set up the loss
        loss_fn=nn.MSELoss()
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-2)
        for t in range(self.training_steps):
            #zero gradients
            optimizer.zero_grad()
            #forward backward optimize
            y_pred=self.forward(training_vectors)
            loss=loss_fn(y_pred,target_vectors)
            loss.backward()
            optimizer.step()


    def find_best_control_signal(self,input_array,target_output):
        guess=0
        eps=0.001
        for i in range(self.optimize_loops):
            y=((self.predict(input_array,guess)-target_output)**2).item()
            yp=((self.predict(input_array,guess+eps)-target_output)**2).item()
            grad=(yp-y)/eps
            if grad==0:
                break
            newguess=guess-y/grad
            guess=newguess
        return guess


