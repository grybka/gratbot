
from Behavior import *
from MotionBehaviors import *
from ExploreBehavior import ExploreBehavior
from ChasingBehavior import TrackIfSeenBehavior
from ChasingBehavior import CalibrateChasingBehavior
from CalibrateMotionBehavior import CalibrateMotionBehavior
from CalibrateMotionBehavior import RandomMotionTrackVisual
from underpinnings.BayesianArray import BayesianArray
import threading
import sys
import numpy as np
import traceback
import logger
import queue

class TextCommandBehavior(GratbotBehavior):

    def __init__(self):
        self.text_input=queue.Queue()
        self.commands=[]
        self.sub_behavior=None

    def interpret_command(self,command,**kwargs):
        tokens=command.split()
        lt=len(tokens)
        try:
            #kwargs["broker"].publish({"timestamp": time.time(),"motor_command": {"lr_throttle": [lt,rt], "duration": dur}},["motor_command"])
            if tokens[0]=="hello":
                print("hello")
            elif tokens[0]=="calibration_motion":
                print("Beginning Calibration Dance")
                self.sub_behavior=CalibrateMotionBehavior()
            else:
                print("Could not process command {}".format(command))
        except Exception as e:
            print("Error processing command {}: {}".format(command,e))
            print("{}".format(traceback.format_exc()))

    def act(self,**kwargs):
        #gprint("act called")
        #broker=kwargs["broker"]
        #textinput=kwargs["text_input"]
        try:
            text=self.textinput.get_nowait()
            self.commands.append(text)
            #gprint("got text {}".format(text))
        except:
            #gprint("{}".format(sys.exc_info()))
            pass #its empty, this is fine

        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.COMPLETED:
                logging.info("Completed Action")
                self.sub_behavior=None
            if step_status==GratbotBehaviorStatus.FAILED:
                logging.info("Failed Action")
                self.sub_behavior=None
        else:
            step_status=GratbotBehaviorStatus.COMPLETED
            self.sub_behavior=None
        if len(self.commands)==0:
            return GratbotBehaviorStatus.INPROGRESS
        to_do=None
        to_do=self.commands.pop(0)
        self.interpret_command(to_do,**kwargs)
        return GratbotBehaviorStatus.INPROGRESS
