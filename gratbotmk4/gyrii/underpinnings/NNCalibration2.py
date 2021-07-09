import torch
from torch.utils.data import Dataset,DataLoader,TensorDataset,random_split
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import random
import numpy as np

class ScatteredLeakyMemory(Dataset):
    def __init__(self,max_len,uncs):
        self.max_len=max_len
        self.experiences=[] #[ [vector], nearest_neighbor_dist, [nearest neighbor vector] ]
        self.uncs=np.array(uncs) #uncertainties used for distance

    def add_experience(self,vector,forget=True):
        min_dist=np.inf
        elem=vector #the closest
        for x in self.experiences:
            dist=np.sum( ((np.array(vector)-np.array(x[0]) )/self.uncs)**2 )
            #print("dist is {}".format(dist))
            if dist<x[1]:  #if I'm adding something closer, update that
                x[1]=dist
            if dist<min_dist:
                min_dist=dist
                elem=x[0]
        self.experiences.append([ vector,min_dist,elem])
        if forget:
            todel=self.forget()
            if todel is not None and todel==vector:
                return False
        return True

    def get_as_batches(self):
        x=[]
        for i in range(len(self)):
            x.append(self[i])
        return torch.stack(x)

    def forget(self):
        if len(self)>self.max_len:
            toresort=[]
            dists=[ x[1] for x in self.experiences ]
            min_ind=dists.index(min(dists))
            todel=self.experiences.pop(min_ind)
            for x in self.experiences:
                if x[2]==todel[0]:
                    toresort.append(x[0])
                    self.experiences.remove(x)
            for x in toresort:
                self.add_experience(x,forget=False)
            return todel
        return None

    def __getitem__(self,index):
        return torch.tensor(self.experiences[index][0]).float()

    def __len__(self):
        return len(self.experiences)


def weighted_mse_loss(inputs, target, weight):
    return torch.sum( ( (inputs - target)/weight ) ** 2)

class StatePredictorWithPolicy:
    #Given initial states, decisions taken, and final states
    #train a predictor to predict final states given initial states and decisions
    #and a policy to get pick the decision that gets you closest to the final state given an initial state
    def __init__(self,predictor,policy,memory_size,initial_unc,decision_unc,final_unc):
        self.predictor=predictor
        self.policy=policy
        self.initial_unc=torch.tensor(initial_unc)
        self.final_unc=torch.tensor(final_unc)
        self.decision_unc=torch.tensor(decision_unc)
        self.initial_len=len(initial_unc)
        self.decision_len=len(decision_unc)
        self.final_len=len(final_unc)
        self.experience_memory=ScatteredLeakyMemory(memory_size,[*initial_unc,*decision_unc,*final_unc])
        self.train_epochs=64
        self.fresh_memories=0

    def observe(self,initial,decision,final):
        if self.experience_memory.add_experience([*initial,*decision,*final]):
            self.fresh_memories+=1

    def predict_decision(self,initial,final):
        function_input=[ *initial,*final]
        predicted_output=self.policy(torch.unsqueeze(torch.tensor(function_input).float(),0))[0]
        return predicted_output.detach().numpy().tolist()



    def train(self):
        self.train_predictor()
        self.train_policy()
        self.fresh_memories=0

    def train_predictor(self):
        loader=DataLoader(self.experience_memory,batch_size=64,shuffle=True)
        optimizer = optim.Adam(self.predictor.parameters(), lr=0.01)
        for epoch in range(self.train_epochs):
            for vector in loader:
                input_vector=vector[:,0:self.initial_len+self.decision_len]
                output_vector=vector[:,self.initial_len+self.decision_len:]
                pred_output_vector=self.predictor( input_vector )
                loss=weighted_mse_loss(output_vector,pred_output_vector,self.final_unc)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

    def train_policy(self):
        loader=DataLoader(self.experience_memory,batch_size=64,shuffle=True)
        optimizer = optim.Adam(self.policy.parameters(), lr=0.01)
        print("training")
        for epoch in range(self.train_epochs):
            for vector in loader:
                input_vector=torch.cat( (vector[:,0:self.initial_len],vector[:,self.initial_len+self.decision_len:]),1 )
                output_vector=vector[:,self.initial_len:self.initial_len+self.decision_len]
                pred_output_vector=self.policy( input_vector )
                loss=weighted_mse_loss(output_vector,pred_output_vector,self.decision_unc)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

    def fantasy_train_policy(self,target_bounds):
        #generate a bunch of scenarios from real starting points, but with different targets
        loader=DataLoader(self.experience_memory,batch_size=64,shuffle=True)
        optimizer = optim.Adam(self.policy.parameters(), lr=0.01)
        for epoch in range(self.train_epochs):
            for vector in loader:
                #generate new target here
                new_targets=[]
                for i in range(vector.shape[0]):
                    newtarget=[]
                    for d in target_bounds:
                        newtarget.append(random.uniform(d[0],d[1]))
                    new_targets.append(torch.tensor(newtarget))
                new_targets=torch.stack(new_targets)

                #the_input=torch.cat( (inputs,targets),1)
                the_input=torch.cat( (vector[:,0:self.initial_len],new_targets),1)
                policy_out=self.policy(the_input)
                predicted_out=self.predictor( torch.cat( (vector[:,0:self.initial_len] ,policy_out),1))
                loss=weighted_mse_loss(predicted_out,new_targets,self.final_unc)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
