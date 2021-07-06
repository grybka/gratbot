#for neural network calibrations
import torch
from torch.utils.data import Dataset,DataLoader,TensorDataset,random_split
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import random
import numpy as np

class WeightedLeakyMemory(Dataset):
    def __init__(self,max_len,weight_bounds=[0,1]):
        self.max_len=max_len
        self.weight_bounds=weight_bounds
        self.experiences=[] #[ input, target, weight]
        self.min_weight=np.inf

    def get_average_weight(self):
        thesum=0
        for x in self.experiences:
            thesum+=x[2]
        return thesum/len(self.experiences)

    def get_average_inv_weight(self):
        thesum=0
        for x in self.experiences:
            thesum+=1/x[2]
        return thesum/len(self.experiences)

    def update_weight(self,index,weight):
        self.experiences[index][2]=weight
        if weight<self.min_weight:
            self.min_weight=weight

    def choose_by_inv_weight(self):
        weights=[ x[2] for x in self.experiences]
        return random.choices(self.experiences, weights=weights)[0]

    def choose_by_weight(self):
        weights=[ 1/x[2] for x in self.experiences]
        return random.choices(self.experiences, weights=weights)[0]

    def forget(self):
        while len(self)>self.max_len:
            self.experiences.remove(self.choose_by_weight())

    def add_experience(self,experience_input,experience_target,weight):
        #your chance of forgetting is propotional to the experience weight
        #only record experience if it has a greater weight than the smallest weights
        if len(self)<self.max_len or weight>self.min_weight:
            if weight<self.min_weight:
                self.min_weight=weight
            self.experiences.append([experience_input,experience_target,weight])
            self.forget()
            return True
        return False

    def get_as_batches(self):
        inp=[]
        tar=[]
        for i in range(len(self)):
            a,b=self[i]
            inp.append(a)
            tar.append(b)
        return torch.stack(inp),torch.stack(tar)

    def __getitem__(self,index):
        return torch.tensor(self.experiences[index][0]).float(),torch.tensor(self.experiences[index][1]).float()

    def __len__(self):
        return len(self.experiences)

class StatePredictor:

    def __init__(self,predictor,loss_bounds=[0.01,1],decision_bounds=[ [-1,1] ],memory_size=64):
        #predictor is a module that given in input state and action state, predicts the output state
        self.predictor=predictor
        self.experience_memory=WeightedLeakyMemory(memory_size)
        self.loss_bounds=loss_bounds
        self.decision_bounds=decision_bounds  #array of [min,max]
        self.train_batch_size=64
        self.train_epochs=64
        self.fresh_memories=0

    def loss_to_weight(self,loss):
        return 1/np.clip(loss,self.loss_bounds[0],self.loss_bounds[1])

    def predict_output(self,input_state,decision):
        function_input=[ *input_state,*decision]
        predicted_output=self.predictor(torch.unsqueeze(torch.tensor(function_input).float(),0))[0]
        return predicted_output.detach().numpy()


    def observe(self,input_state,decision,output_state):
        with torch.set_grad_enabled(False):
            loss_function = torch.nn.MSELoss()
            function_input=[ *input_state,*decision]
            #function_input=torch.cat( (torch.tensor(input_state).float(),torch.tensor(decision).float()),0)
            predicted_output=self.predictor(torch.unsqueeze(torch.tensor(function_input).float(),0))[0]
            loss=loss_function(predicted_output,torch.tensor(output_state).float()).item()
        if self.experience_memory.add_experience(function_input,output_state,self.loss_to_weight(loss)):
            self.fresh_memories+=1

    def decide_random(self,input_state):
        my_decision=[]
        for d in self.decision_bounds:
            my_decision.append(random.uniform(d[0],d[1]))
        return my_decision

    def decide_by_probing(self,input_state,desired_output_state,pts_per_dim):
        pts=[ input_state ]
        for i in range(len(self.decision_bounds)):
            toadd=np.linspace(self.decision_bounds[i][0],self.decision_bounds[i][1],pts_per_dim)
            newpts=[]
            for i in range(len(toadd)):
                for j in range(len(pts)):
                    newpts.append( [*(pts[j]),toadd[i]] )
            pts=newpts
        inputs=torch.tensor(pts).float()
        predictions=self.predictor(inputs)
        loss_function = torch.nn.MSELoss()
        best_inv_loss=0
        best_index=0
        for i in range(predictions.shape[0]):
            loss=loss_function(predictions[i],torch.tensor(desired_output_state).float())
            #print("action {} loss {}".format(inputs[i],loss))
            if 1/loss > best_inv_loss:
                best_index=i
                best_inv_loss=1/loss
        return pts[best_index][len(input_state):]


    def train(self):
        loader=DataLoader(self.experience_memory,batch_size=64,shuffle=True)
        loss_function = torch.nn.MSELoss()
        optimizer = optim.Adam(self.predictor.parameters(), lr=0.01)
        for epoch in range(self.train_epochs):
            for inputs,targets in loader:
                out=self.predictor( inputs )
                loss=loss_function(out,targets )
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        #now recalculate everything in memory
        with torch.set_grad_enabled(False):
            inputs,targets=self.experience_memory.get_as_batches()
            out=self.predictor( inputs )
            for i in range(len(self.experience_memory)):
                self.experience_memory.update_weight(i,self.loss_to_weight(loss_function(out[i],targets[i]).item()))
        self.fresh_memories=0
        return loss_function(out,targets).item()/len(self.experience_memory)
