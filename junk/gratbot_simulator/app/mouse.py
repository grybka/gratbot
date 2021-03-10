from MouseModel import MouseWorld
from mouse_network import MouseNetworkReplayMemory
from mouse_network import MouseNetwork
from mouse_network import MouseNetwork_Surprise
import cv2
import numpy as np
import os

def log_to_file(filename,mystring):
    f=open(filename,'a+')
    f.write(mystring)
    f.close()
    
log_file_name="my_log.x"
try:
    os.remove(log_file_name)
except OSError as error:
    pass

def index_to_onehot(index,options):
    ret=np.zeros(len(options))
    ret[index]=1.0
    return ret

world=MouseWorld()
world.add_boundary(3.0,3.0)
world.add_random_cheese()
world.add_corner_markers(3.0,3.0)
#first step to identify what's going on
control_vector=[0,0]
control_options=[ [0,0] ,
                  [0.5,0] ,
                  [-0.5,0] ,
                  [0,0.5] ,
                  [0,-0.5] ,
                  [0.5,0.5],
                  [-0.5,0.5],
                  [-0.5,-0.5],
                  [0.5,-0.5]]

observation,reward=world.step(control_vector)

##Network here
memory=MouseNetworkReplayMemory(20,20)
network=MouseNetwork_Surprise(len(observation),len(control_options),max(len(observation),1))


#control_vector=np.array([0,0])
#main loop
nsteps=200

prev_observation=None
prev_action=None
#for step in range(nsteps):
counter=1
action_index=0
while True:
    counter+=1
#observation,reward=world.step([0.1,1.0])
    observation,reward=world.step(control_vector)

    if counter % 1000==0:
        print("counter {}".format(counter))
        print("prev observation {}".format(prev_observation))
        print("next observation {}".format(observation))
        print("predicted {}".format(next_state_pred))
        print("action: {}".format(control_vector))
        c_pred,c_state=network.predictor.predict_single(prev_observation,observation,index_to_onehot(action_index,control_options))
        print("predicted action: {}".format(c_pred))

#print(observation)
#print(reward)
    #add episode to memory
    if prev_observation is not None:
        if not memory.remember(prev_observation,index_to_onehot(action_index,control_options),observation,reward):
#print("Training!")
            items=np.zeros(3)
            for k in range(5):
                ret=network.train(memory)
                for j in range(len(ret)):
                    items[j]+=ret[j]
            table=""
            for i in range(len(items)):
                table=table+" {}".format(items[i])
            log_to_file(log_file_name,table+"\n")
            memory.new_episode()
    prev_observation=observation
#print(observation)
    #print(len(memory.episodes),len(memory.episodes[memory.on_episode]))
# world.render()
    #figure out next move
    control_vector_hot,next_state_pred,reward_pred=network.select_action(observation,control_options)
    action_index=np.argmax(control_vector_hot)
    control_vector=control_options[action_index]
#print("target: {}".format(control_vector))

    key=cv2.waitKey(50)
#key=cv2.waitKey(0)
    #82 = forward
    #84 = backward
    #81 = left
    #83 = right
    if key==-1:
        x=1
        #noop
    elif key==82:
        control_vector[0]+=0.1
        if control_vector[0]>1:
            control_vector[0]=1
    elif key==84:
        control_vector[0]-=0.1
        if control_vector[0]<-1:
            control_vector[0]=-1
    elif key==81:
        control_vector[1]+=0.1
        if control_vector[1]>1:
            control_vector[1]=1
    elif key==83:
        control_vector[1]-=0.1
        if control_vector[1]<-1:
            control_vector[1]=-1
    elif key==113:
        print("Quit pressed")
        break
    else:
        print("key {}".format(key))
        break
    
#print("key {}".format(key))
#print("{} {}".format(world.mouse_pos[0],world.mouse_pos[1]))
