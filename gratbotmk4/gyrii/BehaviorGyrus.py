
from Gyrus import ThreadedGyrus
from behaviors.Behavior import GratbotBehaviorStatus
import time
import logging
from gyrii.behaviors.ChaseBehavior import TrackIfSeen
from gyrii.behaviors.CalibrateMotionBehavior import ExerciseServo
from gyrii.behaviors.FollowBehavior import find_and_follow, RunMotors

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#this is an awkward place to put this, but the behavior needs some sense of "state"
def update_state(message,state):
    if "servo_response" in message:
        m=message["servo_response"]
        if "servo_angle" not in state:
            state["servo_angle"]={}
        state["servo_angle"][m["servo_number"]]=m["angle"]

class BehaviorGyrus(ThreadedGyrus):
    def __init__(self,broker,on_behavior,pass_kwargs={}):
        self.short_term_memory={}
        self.state={}
        self.on_behavior=on_behavior
        self.my_kwargs=pass_kwargs
        self.skip_until_time=0
        super().__init__(broker)


    def get_keys(self):
        return ["clock_pulse","tracks","behavior_request","servo_response","rotation_vector","command_received"]

    def get_name(self):
        return "BehaviorGyrus"

    def read_message(self,message):
        #Short term memory
        for key in message:
            if key=="timestamp": #don't need to save that
                continue
            self.short_term_memory[key]=message
        update_state(message,self.state)
        if "clock_pulse" in message:
            if message["timestamp"]>self.skip_until_time:
                self.execute_behavior()
        if "behavior_request" in message: #stop what you're doing.  do something else
            self.on_behavior=None #stops doing whatever
            if message["behavior_request"]["name"]=="trackifseen":
                logger.info("Track If Seen Behavior Triggered")
                self.on_behavior=TrackIfSeen()
            if message["behavior_request"]["name"]=="exerciseservo":
                logger.info("Exercise servo Triggered")
                self.on_behavior=ExerciseServo()
            elif message["behavior_request"]["name"]=="nothing":
                logger.info("Behavior set to None")
                ...
            else:
                logging.warning("Got invalid behavior request: {}".format(message["behavior_request"]))
        if "command_received" in message: #used for verbal commands.  Interrupt in behavior later?
            command=message["command_received"]["command"]
            if message["command_received"]["confidence"]<0.7:
                logging.info("Confidence too low, ignoring command that could have been {}".format(command))
                return
            if command=="right":
                logger.info("right")
                self.on_behavior=RunMotors(0.5,-0.5,0.5)
            if command=="left":
                logger.info("left")
                self.on_behavior=RunMotors(-0.5,0.5,0.5)
            if command=="come":
                logger.info("Find and follow")
                self.on_behavior=find_and_follow(["person","face"])
            if command=="heel":
                logger.info("Find and follow")
                self.on_behavior=find_and_follow(["person","face"])
            if command=="stop":
                logger.info("Stop")
                self.on_behavior=None

    def execute_behavior(self):
        #returns a list of messages
        if self.on_behavior==None:
            return []
        the_kwargs=self.my_kwargs
        the_kwargs["short_term_memory"]=self.short_term_memory
        the_kwargs["state"]=self.state
        the_kwargs["broker"]=self.broker
        #resp=self.on_behavior.act(short_term_memory=self.short_term_memory,message_queue=message_queue)
        resp,_=self.on_behavior.act(**the_kwargs)
        if resp==GratbotBehaviorStatus.COMPLETED:
            logger.warning("Behavior Completed.  Halting")
            self.on_behavior=None
        if resp==GratbotBehaviorStatus.COMPLETED:
            logger.warning("Behavior Failed.  Halting")
            self.on_behavior=None
        self.skip_until_time=time.time()
