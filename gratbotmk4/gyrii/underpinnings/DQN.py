import torch
from torch.utils.data import Dataset,DataLoader,TensorDataset,random_split
from collections import deque
import numpy as np
import torch.optim as optim


class DQN_Episode(Dataset):
    def __init__(self,states,actions,rewards,input_state_length=1,reward_decay_factor=0.5):
        #states, actions, and rewards are lists of torch tensors
        self.states=torch.stack(states)
        self.actions=torch.stack(actions)
        self.rewards=torch.stack(rewards)
        self.input_state_length=input_state_length
        self.reward_decay_factor=reward_decay_factor
        self.end_padding=int(1/(1-reward_decay_factor)+1)
        self.calculate_q_estimates() #this

    def calculate_q_estimates(self):
        #this is the q value assuming that the choices made were the best ones
        on_Q=self.rewards[-1] #presume the last reward is stuck
        q_backwards=[]
        for i in reversed(range(len(self.rewards))):
            on_Q=on_Q*(1-self.reward_decay_factor)+self.rewards[i]*self.reward_decay_factor
            q_backwards.append(on_Q)
        self.qs=torch.stack(list(reversed(q_backwards))).float()

    def __len__(self):
        return max(self.states.shape[0]-self.input_state_length-self.end_padding,0)

    def __getitem__(self,index):
        #returns:
        #the last input_state_length states ending with this state
        #the action taken at the last state
        #the last_input_state_length states ending with next state
        #the reward at the next state
        #the naiive q estimate of the next state
        return self.states[index:index+self.input_state_length,:],self.actions[index+self.input_state_length-1],self.states[index+1:index+self.input_state_length+1],self.rewards[index+self.input_state_length],self.qs[index+self.input_state_length]

class DQN_Replay_Memory:
    def __init__(self,max_size=40):
        self.episodes=[]
        self.max_size=max_size

    def add_episode(self,episode,loss):
        self.episodes.append([episode,loss])
        self.relign_to_size()

    def relign_to_size(self):
        while len(self.episodes)>self.max_size:
            losses=[ x[1] for x in self.episodes ]
            #self.episodes.pop(np.random.choice(np.arange(len(self.episodes))))
            if np.random.rand()<0.99:
                smallest=np.argmin(losses)
                self.episodes.pop(smallest)
            else:
                largest=np.argmax(losses)
                self.episodes.pop(largest)





class DQN_Manager:
    def __init__(self,predictor,n_actions=2,episode_length=60,memory_episodes=40):
        self.predictor=predictor
        self.predictor_n_steps_behind=3
        self.n_actions=n_actions
        if memory_episodes>0:
            self.memory=DQN_Replay_Memory(max_size=memory_episodes)
        else:
            self.memory=None

        self.episode_length=episode_length #how long an episode is in steps
        #temporary storage while assembling state,action,reward triple
        self.states=[]
        self.rewards=[]
        self.actions=[]

        #has something changed that warrants a new training session
        self.ready_to_train=False

    def observe_state(self,state,reward):
        self.states.append(state)
        self.rewards.append(reward.float())

    def observe_action(self,action):
        #this is the state I just observed
        #the reward based off the state I just observed
        #and the action I choose to take based on it
        self.actions.append(action)
        #TODO package and send to memory
        if len(self.states)>self.episode_length:
            new_episode=DQN_Episode(self.states,self.actions,self.rewards,input_state_length=self.predictor_n_steps_behind)
            loss=self.get_episode_loss(new_episode,torch.nn.MSELoss(),bellman_fraction=0.5)
            if self.memory is not None:
                self.memory.add_episode(new_episode,loss)
            self.states=self.states[-self.predictor_n_steps_behind:]
            self.actions=self.actions[-self.predictor_n_steps_behind:]
            self.rewards=self.rewards[-self.predictor_n_steps_behind:]
            self.ready_to_train=True

    def break_episode(self): #call if there is a gap in usable data to store episode and reset
        new_episode=DQN_Episode(self.states,self.actions,self.rewards,input_state_length=self.predictor_n_steps_behind)
        if len(new_episode)>0:
            loss=self.get_episode_loss(new_episode,torch.nn.MSELoss(),bellman_fraction=0.5)
            if self.memory is not None:
                self.memory.add_episode(new_episode,loss)
        self.states=[]
        self.actions=[]
        self.rewards=[]
        self.ready_to_train=True

    def choose_action(self,random_chance):
        if len(self.states)<self.predictor_n_steps_behind:
            return 0
        if np.random.rand()<random_chance:
            return np.random.choice(np.arange(self.n_actions))
        #set up for predictor
        with torch.no_grad():
            input_states=torch.stack(self.states[-self.predictor_n_steps_behind:]).unsqueeze(0)
            qs=self.calc_next_action_qs(input_states).detach().numpy()
            chosen_action=np.argmax(qs)
            #chosen_action=np.random.choice(np.arange(self.n_actions),p=probs)
        return chosen_action

    def calc_next_action_qs(self,input_states):
        #input states should have dimensions [batch,sequence,feature]
        with torch.no_grad():
            input_states.unsqueeze(0)
            states=input_states.expand(self.n_actions,-1,-1,-1)
            #print("states shape now {}".format(states.shape))
            actions_by_number=torch.tensor(np.arange(self.n_actions)).long()
            actions=torch.nn.functional.one_hot(actions_by_number,num_classes=self.n_actions).unsqueeze(1).expand(-1,states.shape[1],-1)
            #print("actions shape now {}".format(actions.shape))
            action_qs=self.predictor.forward(states,actions)
            #print("action weights now {}".format(action_weights.shape))
            #smax=torch.nn.Softmax(dim=0)
            #probs=smax(action_weights).squeeze().detach().numpy()
            return action_qs

    def train_predictor(self,n_epochs=2):
        loss_function = torch.nn.MSELoss()
        optimizer = optim.Adam(self.predictor.parameters(), lr=0.01)
        bellman_fraction=0.5
        loss_log=[]
        for epoch in range(n_epochs):
            #print("on epoch {}".format(epoch))
            loss_sum=0
            for i in range(len(self.memory.episodes)):
                #print("on episode {}".format(i))
                episode=self.memory.episodes[i][0]
                this_loss=self.get_episode_loss(episode,loss_function)
                self.memory.episodes[i][1]=this_loss.item() #update the stored loss
                loss_sum=this_loss+loss_sum
            #print("loss sum type {}".format(loss_sum.type()))
            optimizer.zero_grad()
            loss_sum.backward()
            loss_log.append(loss_sum.item()/len(self.memory.episodes))
            optimizer.step()
        self.ready_to_train=False
        return loss_log

    def get_episode_loss(self,episode,loss_function,bellman_fraction=0.5):
        loss_sum=0
        loader=DataLoader(episode,batch_size=64,shuffle=False)
        for states,actions,next_states,rewards,q_estimates in loader:
            actions=torch.nn.functional.one_hot(actions,num_classes=self.n_actions).float()
            rewards=rewards.unsqueeze(-1)
            q_estimates=q_estimates.unsqueeze(-1)
            #what q do I predict
            #print("states shape {}".format(states.shape))
            #print("actions shape {}".format(actions.shape))
            #print("q_estimates shape {}".format(q_estimates.shape))
            #print("states type {}".format(states.type()))
            #print("actions type {}".format(actions.type()))
            q_guess=self.predictor(states,actions)
            #print("q_guess shape {}".format(q_guess.shape))
            #print("rewards shape {}".format(rewards.shape))
            #print("q_estimates type {}".format(q_estimates.type()))
            #print("q guess type {}".format(q_guess.type()))
            #get the bellman q and expected q
            next_action_qs=self.calc_next_action_qs(next_states)
            #print("next action qs shape {}".format(next_action_qs.shape))
            best_action_qs=torch.max(next_action_qs,dim=0)[0]
            #print("best action qs shape {}".format(best_action_qs.shape))
            q_bellman=rewards*episode.reward_decay_factor+best_action_qs*(1-episode.reward_decay_factor)
            #print("q bellman shape {}".format(q_bellman.shape))
            q_target=bellman_fraction*q_bellman+(1-bellman_fraction)*q_estimates
            #print("q target shape {}".format(q_target.shape))
            this_loss_sum=loss_function(q_target,q_guess)
            #print("this loss sum type {}".format(this_loss_sum.type()))
            loss_sum=this_loss_sum+loss_sum
        return loss_sum
