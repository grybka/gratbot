from Behavior import GratbotBehavior
from Behavior import GratbotBehaviorStatus
import time

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
