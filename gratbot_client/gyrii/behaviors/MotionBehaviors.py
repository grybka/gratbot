from Behavior import GratbotBehavior
from Behavior import GratbotBehaviorStatus
from gyrii.underpinnings.GratbotLogger import gprint,gprint_low
from underpinnings.BayesianArray import BayesianArray
import numpy as np
import time

class RunMotors(GratbotBehavior):
    def __init__(self,lmotor,rmotor,duration):
        self.lmotor=lmotor
        self.rmotor=rmotor
        self.duration=duration
    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [self.lmotor,self.rmotor], "duration":self.duration } },"motor_command")
        return GratbotBehaviorStatus.COMPLETED

class MotorFixedAmount(GratbotBehavior):
    def __init__(self,fixed_amount,command_type):
        #command type is "turn" or "ahead"
        #fixed amount is actually how many seconds you want the motors on, with sign indicating direction
        self.command_type=command_type
        self.fixed_amount=fixed_amount

    def act(self,**kwargs):
        #Given comms and fused sensors, do something
        #return status
        #status is gratbotbehaviorstatus above (inprogress will be called again and ignore the rest)
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"motor_command": {"type": self.command_type,"magnitude": self.fixed_amount}},"motor_command")
        return GratbotBehaviorStatus.COMPLETED

    def reset(self):
        #in case I want to call this twice, reset to as if it had just been started
        pass

class Turn(GratbotBehavior):
    def __init__(self,angle):
        self.angle=angle

    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"move_command": {"type": "turn","angle": self.angle}},"move_command")
        return GratbotBehaviorStatus.COMPLETED


class MoveAhead(GratbotBehavior):
    def __init__(self,distance):
        self.distance=distance #in meters

    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"move_command": {"type": "ahead","distance": self.distance}},"move_command")
        return GratbotBehaviorStatus.COMPLETED


class Slew(GratbotBehavior):
    def __init__(self,distance):
        self.distance=distance #in meters

    def act(self,**kwargs):
        broker=kwargs["broker"]
        broker.publish({"timestamp": time.time(),"move_command": {"type": "slew","distance": self.distance}},"move_command")
        return GratbotBehaviorStatus.COMPLETED


class AbortIfUltrasonicTooClose(GratbotBehavior):
    def __init__(self,ultrasonic_panic_reading,behavior):
        self.behavior=behavior
        self.ultrasonic_panic_reading=ultrasonic_panic_reading

    def act(self,**kwargs):
        if "ultrasonic_sensor/last_measurement" not in kwargs["short_term_memory"]:
            gprint("waiting for ultrasonic measurement before proceeding")
            return GratbotBehaviorStatus.INPROGRESS
        dist=kwargs["short_term_memory"]["ultrasonic_sensor/last_measurement"]["ultrasonic_sensor/last_measurement"]["average_distance"]
        if dist<self.ultrasonic_panic_reading:
            gprint_low("Aborting behavior, ultrasonic read {}".format(dist))
            return GratbotBehaviorStatus.FAILED
        return self.behavior.act(**kwargs)

class WaitForMotorsOff(GratbotBehavior):
    def __init__(self):
        ...
    def act(self,**kwargs):
        if "drive/motors_active" in kwargs["short_term_memory"]:
            d=kwargs["short_term_memory"]["drive/motors_active"]["drive/motors_active"]
            if d[0]==0 and d[1]==0 :
                return GratbotBehaviorStatus.COMPLETED
            else:
                gprint("motors still active")
        else:
            gprint("no motors active in memory")
        return GratbotBehaviorStatus.INPROGRESS

class WaitForStablePose(GratbotBehavior):
    def __init__(self):
        pass

    def act(self,**kwargs):
        if "latest_pose" in kwargs["short_term_memory"]:
            posemessage=kwargs["short_term_memory"]["latest_pose"]
            if posemessage["pose_notes"]=="pose_is_stable":
                return GratbotBehaviorStatus.COMPLETED
        return GratbotBehaviorStatus.INPROGRESS

class FollowPath(GratbotBehavior):
    def __init__(self,path,desired_accuracy=0.05):
        self.path_remaining=path
        self.sub_behavior=None
        self.close_enough_distance=0.25
        self.max_move=0.25
        self.wait_until=0
        self.command_wait=0.15
        self.desired_accuracy=desired_accuracy

    def move_towards_point(self,point,last_pose,**kwargs):
        delta=point-last_pose.vals[0:2]
        lendelta=np.linalg.norm(delta)
        if lendelta>self.max_move:
            delta=delta*(self.max_move/lendelta)
        vector=[ delta[0],delta[1],0]
        gprint("motion vector {}".format(vector))
        kwargs["broker"].publish({"timestamp": time.time(),"move_command": {"type": "complex","distance": vector}},"move_command")

    def act(self,**kwargs):
        #gprint("FollowPath started")
        broker=kwargs["broker"]
        shared_objects=kwargs["shared_objects"]
        step_status=GratbotBehaviorStatus.COMPLETED
        #if self.sub_behavior is not None:
        #    step_status=self.sub_behavior.act(**kwargs)
        #else:
        #    step_status=GratbotBehaviorStatus.COMPLETED
        if step_status==GratbotBehaviorStatus.COMPLETED:
            if len(self.path_remaining)==0:
                #gprint("Followpath done")
                return GratbotBehaviorStatus.COMPLETED
            motors_active=kwargs["short_term_memory"]["drive/motors_active"]["drive/motors_active"]
            if time.time()<self.wait_until: #If my motors should still be running, do not issue new command
                return GratbotBehaviorStatus.INPROGRESS
            if motors_active[0]!=0 or motors_active[1]!=0 or motors_active[2]!=0: #if my motors are actualy running, do not issue new command
                #TODO if I drop this, will it make my motion smoother?
                return GratbotBehaviorStatus.INPROGRESS
            latest_pose=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose"]["latest_pose"])
            if np.sqrt(latest_pose.covariance[0][0]+latest_pose.covariance[1][1])>self.desired_accuracy:
                # if I don't know where i am to the desired accuracy, don't move
                return GratbotBehaviorStatus.INPROGRESS
            dist_from_node=np.linalg.norm(latest_pose.vals[0:2]-self.path_remaining[0])
            #check if at first location of path
            if dist_from_node>self.close_enough_distance:
                #gprint("moving towards {}".format(self.path_remaining[0]))
                self.move_towards_point(self.path_remaining[0],latest_pose,**kwargs)
                self.wait_until=time.time()+self.command_wait
            else:
                #gprint("close enough, next node")
                self.path_remaining.pop(0)
        return GratbotBehaviorStatus.INPROGRESS

class NavigateToNode(GratbotBehavior):
    def __init__(self,target_node,desired_accuracy=0.05):
        self.desired_accuracy=desired_accuracy
        self.target_node=target_node
        self.sub_behavior=None
        self.path=None

    def find_path(self,shared_objects,target_node,latest_pose):
        pos_path=[]
        node_path=[]
        with shared_objects.locks["graph_map"]:
            graphmap=shared_objects.objects["graph_map"]
            starting_node=graphmap.get_node_nearest_to(latest_pose.vals[0:2])
            try:
                path_nodes=graphmap.astar_path(starting_node,target_node)
            except:
                gprint("No path found, failing")
                return None, None
            for node in path_nodes:
                node_path.append(node)
                pos_path.append(graphmap.graph.nodes[node]['pos'])
        return node_path,pos_path


    def act(self,**kwargs):
        #gprint("navigate to node started")
        broker=kwargs["broker"]
        shared_objects=kwargs["shared_objects"]
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
        else:
            step_status=GratbotBehaviorStatus.COMPLETED
        if step_status==GratbotBehaviorStatus.COMPLETED:
            if self.path is None:
                latest_pose=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose"]["latest_pose"])
                node_path,pos_path=self.find_path(shared_objects,self.target_node,latest_pose)
                if node_path is None:
                    return GratbotBehaviorStatus.FAILED
                self.path=node_path
                gprint("path is {}".format(node_path))
                self.sub_behavior=AbortIfUltrasonicTooClose(0.15,FollowPath(pos_path))
                step_status=GratbotBehaviorStatus.INPROGRESS
        #gprint("navigate to node returning {}".format(step_status))
        return step_status
