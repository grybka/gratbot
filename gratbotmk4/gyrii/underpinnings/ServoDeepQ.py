import underpinnings.DQN as DQN
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import logging
import math
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

#pitch from acceleration vector
#pitch 0 should be z forward, Y down
def calc_pitch(vec):
    x=vec[0]
    y=vec[1]
    z=vec[2]
    xymag=np.sqrt(x*x+y*y)
    return np.arctan2(z,xymag)

class MessagesToNNState:
    def __init__(self):
        self.last_state_emission=0
        self.state_emission_interval=0.1

        self.yaws=[]
        self.rolls=[]
        self.pitches=[]

        self.n_servos=1
        self.servo_neuron_map={0:0,1:1}
        self.servo_vals=np.zeros(self.n_servos)
        self.last_servo_vals=np.zeros(self.n_servos)

        self.servo_command_delta_vals=np.zeros(self.n_servos)

        self.num_pixels=5
        self.x_pixel_locs=np.linspace(0,1,self.num_pixels)
        self.y_pixel_locs=np.linspace(0,1,self.num_pixels)
        self.x_pixels=[]
        self.y_pixels=[]

    def detections_to_nn(self,detections):
        x_pixels=np.zeros(self.num_pixels)
        y_pixels=np.zeros(self.num_pixels)
        for d in detections:
            bbox=d["bbox_array"] #this ranges from 0 to 1, x1,x2,y1,y2
            x=0.5*(bbox[0]+bbox[1])
            wx=bbox[1]-bbox[0]
            y=0.5*(bbox[2]+bbox[3])
            wy=bbox[3]-bbox[2]
            these_x_pixels=np.exp(-((x-self.x_pixel_locs)/wx)**2)
            these_y_pixels=np.exp(-((y-self.y_pixel_locs)/wy)**2)
            x_pixels=np.maximum(x_pixels,these_x_pixels)
            y_pixels=np.maximum(y_pixels,these_y_pixels)
        return x_pixels,y_pixels



    def read_message(self,m):
        if "detections" in m:
            x_pixels,y_pixels=self.detections_to_nn(m["detections"])
            self.x_pixels.append(x_pixels)
            self.y_pixels.append(y_pixels)
        elif "packets" in m:
            #calculate pitch directly from acceleration
            accels=[]
            for i in range(len(m["packets"])):
                accels.append(m["packets"][i]["acceleration"])
            accels=np.stack(accels)
            pitch=calc_pitch(np.mean(accels,axis=0))
            self.rolls.append(0)
            self.yaws.append(0)
            self.pitches.append(pitch)

            timestamp=m["packets"][-1]["rotation_vector_timestamp"]

            #this was for the rotation vector which seems to shift
            #x,y,z,w,acc=m["packets"][-1]["rotation_vector"]
            #roll,pitch,yaw=euler_from_quaternion(x,y,w,z)
            #this is where I correct that the thing is on sideways
            #self.rolls.append(roll)
            #self.pitches.append(yaw) #pi/2 is about level with ground
            #self.yaws.append(pitch)
            if timestamp-self.last_state_emission>self.state_emission_interval:
                #this is when I emit a state
                self.last_state_emission=timestamp
                toret=self.get_neurons_and_reset()
                return toret
        elif "servo_response" in m:
            if m["servo_response"]["servo_number"] in self.servo_neuron_map:
                self.servo_vals[self.servo_neuron_map[m["servo_response"]["servo_number"]]]=m["servo_response"]["angle"]
                #print(self.servo_vals)
            ...
        elif "servo_command" in m:
            if m["servo_command"]["servo_number"] in self.servo_neuron_map:
                if "delta_angle" in m["servo_command"]:
                    i=self.servo_neuron_map[m["servo_command"]["servo_number"]]
                    self.servo_command_delta_vals[i]+=m["servo_command"]["delta_angle"]
        return None


    def get_neurons_and_reset(self):
        #returns (n+x) neurons as numpy array
        av_yaw=np.mean(self.yaws)
        av_roll=np.mean(self.rolls)
        av_pitch=np.mean(self.pitches)
        if len(self.y_pixels)!=0:
            av_ypixels=np.mean(self.y_pixels,axis=0)
        else:
            av_ypixels=np.zeros(self.num_pixels)
        delta_servos=90*(self.servo_vals-self.last_servo_vals)
        self.yaws=[]
        self.pitches=[]
        self.rolls=[]
        self.x_pixels=[]
        self.y_pixels=[]


        ret={}
        ret["yaw"]=[av_yaw/np.pi]
        ret["roll"]=[av_roll/np.pi]
        ret["pitch"]=[av_pitch/np.pi]
        ret["ypixels"]=av_ypixels.copy()
        ret["servo_response"]=self.servo_vals.copy()/180
        ret["servo_deltas"]=delta_servos.copy()
        ret["commands"]=self.servo_command_delta_vals
        #print(ret["servo_response"])
        #ret=np.array([av_yaw/np.pi,*av_ypixels,*self.servo_vals,*delta_servos])
        #commands=[*(self.servo_command_delta_vals/2)]
        self.last_servo_vals=self.servo_vals.copy()
        #self.servo_vals=np.zeros(self.n_servos)
        self.servo_command_delta_vals=np.zeros(self.n_servos)

        return ret

    def get_uncertainties(self):
        ret={}
        ret["yaw"]=0.01
        #TODO use this to give weights when training multiple things
        return ret

def assemble_vector(mydict,labels):
    ret=[]
    for label in labels:
        ret.extend(mydict[label])
    return np.array(ret)

def calc_reward(mydict):
    #print("length of ypixels {}".format(len(mydict["ypixels"])))
    middle=len(mydict["ypixels"])//2
    #print("middle is {}".format(middle))
    looking_reward=0.9*mydict["ypixels"][middle]+0.05*mydict["ypixels"][middle-1]+0.05*mydict["ypixels"][middle+1]
    servo_neck_strain_max=135/180
    servo_neck_strain_min=85/180
    neck_strain_reward=0
    x=mydict["servo_response"]
    if x<servo_neck_strain_min:
        neck_strain_reward=float(-10*(x-servo_neck_strain_min)**2)
    if x>servo_neck_strain_max:
        neck_strain_reward=float(-10*(x-servo_neck_strain_max)**2)
    return looking_reward+neck_strain_reward


class DeepQPredictor(nn.Module):
    def __init__(self,input_size,control_size,n_steps_back):
        #we will presume that the input includes the control vector

        super().__init__()
        self.network=nn.Sequential(nn.Linear(input_size*n_steps_back+control_size,input_size),
                                   nn.Sigmoid(),
                                   nn.Linear(input_size,input_size//2),
                                   nn.Sigmoid(),
                                   nn.Linear(input_size//2,1))

    def forward(self,input_vectors,control):
        #print("input vectors shape {}".format(input_vectors.shape))
        input_vectors_flattened=torch.flatten(input_vectors,start_dim=-2)
        #print("input_vectors_shape flattened {}".format(input_vectors_flattened.shape))
        #print("controles shape {}".format(control.shape))
        my_input=torch.cat([input_vectors_flattened,control],dim=-1)
        return self.network(my_input)

class ServoTrackerAgent:
    def __init__(self,episode_length=20):
        self.n_actions=3
        predictor=DeepQPredictor(9,self.n_actions,3)
        self.message_to_nn_state=MessagesToNNState()
        self.agent=DQN.DQN_Manager(predictor,episode_length=episode_length,memory_episodes=-1,n_actions=self.n_actions)
        self.state_labels=["pitch","servo_response","servo_deltas","ypixels","commands"]
        self.last_action_choice=0
        self.random_motion_chance=0.2
        #self.random_motion_chance=1.0
        #self.random_motion_chance=0
        self.action_servo_map=[0,-2,2]
        self.servo_num=0

    def read_message(self,message,broker):
        if "ServoTrackerAgentNewWeights" in message:
            self.agent.predictor.load_state_dict(message["ServoTrackerAgentNewWeights"])
        else:
            #logger.debug("message passed {}".format(message["keys"]))
            state=self.message_to_nn_state.read_message(message)
            if state is None:
                return
            state_vector=assemble_vector(state,self.state_labels)
            #logger.debug("state vector shape {}".format(state_vector.shape))
            reward=calc_reward(state)
            #logger.debug("reward {}".format(reward))

            self.agent.observe_state(torch.tensor(state_vector).float(),torch.tensor(reward))
            if np.random.rand()<self.random_motion_chance:
                next_action=self.reflex_action(state)
            else:
                next_action=self.agent.choose_action(self.random_motion_chance)

            #logger.debug("action chosen {} ({})".format(next_action,type(next_action)))
            self.agent.observe_action(torch.tensor(next_action))
            servo_command={"timestamp": time.time(),"servo_command": {"servo_number": self.servo_num,"delta_angle": self.action_servo_map[next_action]}}
            if next_action != 0:
                broker.publish(servo_command,"servo_command")
            broker.publish({"timestamp": time.time(),"ServoTrackerState": [state_vector,reward,next_action]},["ServoTrackerState"])

    def reflex_action(self,state): #when I need a 'random' action
        #logger.debug("pitch, yaw, roll is {:.2f} {:.2f} {:.2f}".format(state["pitch"][0],state["yaw"][0],state["roll"][0]))
        #return 0
        #reflex action is to move in the direction of detection if detection available
        det_neurons=state["ypixels"]
        det_sum=np.sum(det_neurons)
        if det_sum==0: #no detection
            if state["pitch"][0]<0.1:
                #looking kinda down
                #logger.debug("looking kind of down {:.2f}".format(state["pitch"][0]))
                logger.debug("reflex empty pitch {}, servo {}, move up".format(state["pitch"][0],state["servo_response"]))
                return np.random.choice(np.arange(3,dtype=np.int64),p=[0.2,0.6,0.2])
            elif state["pitch"][0]>0.20:
                #logger.debug("looking kind of up {:.2f}".format(state["pitch"][0]))
                #looking kinda up
                logger.debug("reflex empty pitch {}, servo {}, move down".format(state["pitch"][0],state["servo_response"]))
                return np.random.choice(np.arange(3,dtype=np.int64),p=[0.2,0.2,0.6])
            else:
                return np.random.choice(np.arange(3,dtype=np.int64),p=[0.2,0.4,0.4])
        det_wsum=np.sum(np.arange(0,5)*det_neurons)
        av_n=det_wsum/det_sum
        return np.random.choice(np.arange(3,dtype=np.int64),p=[0.4,0.3,0.3])
        #if av_n<2:
        #    return np.random.choice(np.arange(3,dtype=np.int64),p=[0.2,0.6,0.2])
        #else:
        #    return np.random.choice(np.arange(3,dtype=np.int64),p=[0.2,0.2,0.6])


class ServoTrackerLearner:
    def __init__(self,episode_length=20,memory_episodes=30,n_training_epochs=10):
        self.n_training_epochs=n_training_epochs
        self.n_actions=3
        predictor=DeepQPredictor(9,self.n_actions,3)
        self.agent=DQN.DQN_Manager(predictor,episode_length=episode_length,memory_episodes=memory_episodes,n_actions=self.n_actions)
        self.save_counter=0

    def read_message(self,message,broker):
        #returns state,reward, action if new state message, otherwise nothing
        if "ServoTrackerState":
            state=message["ServoTrackerState"][0]
            reward=message["ServoTrackerState"][1]
            action=message["ServoTrackerState"][2]
            self.agent.observe_state(torch.tensor(state).float(),torch.tensor(reward).float())
            self.agent.observe_action(torch.tensor(action).long())
            if self.agent.ready_to_train:
                start_time=time.time()
                loss_log=self.agent.train_predictor(self.n_training_epochs)
                #logger.debug("train time {} loss {}".format(time.time()-start_time,loss_log[-1]))
                print("train time {} loss {}".format(time.time()-start_time,loss_log[-1]))
                self.broadcast_trained_state(broker)
                #self.save_counter+=1
                #if self.save_counter>4:
                #    self.dump_memory()
                #    self.save_counter=0
            return torch.tensor(state).float(),torch.tensor(reward).float(),torch.tensor(action).long()
        return None,None,None

    def broadcast_trained_state(self,broker):
        message={"timestamp": time.time(),"ServoTrackerAgentNewWeights": self.agent.predictor.state_dict() }
        broker.publish(message,"ServoTrackerAgentNewWeights")

    def dump_memory(self):
        to_save={}
        epidose=[]
        for episode,loss in self.agent.memory.episodes:
            epidose.append([episode.states,episode.actions,episode.rewards])
        to_save["episodes"]=epidose
        to_save["predictor"]=self.agent.predictor.state_dict()
        torch.save(to_save,'servo_memory_dump.pt')

    def load_from_file(self,fname):
        state_dict=torch.load(fname)
        self.agent.predictor.load_state_dict(state_dict)
        logger.debug("Loaded from file {}".format(fname))
