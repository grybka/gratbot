
from Behavior import *
from MotionBehaviors import *
from ExploreBehavior import ExploreBehavior
from ChasingBehavior import TrackIfSeenBehavior
from CalibrateMotionBehavior import CalibrateMotionBehavior
from CalibrateMotionBehavior import RandomMotionTrackVisual
from underpinnings.BayesianArray import BayesianArray
from gyrii.underpinnings.GratbotLogger import gprint,gprint_low
import threading
import sys
import numpy as np
import traceback

class TextCommandBehavior(GratbotBehavior):

    def __init__(self,gridmap=None):
        self.commands=[]
        self.sub_behavior=None

    def interpret_command(self,command,**kwargs):
        tokens=command.split()
        lt=len(tokens)
        try:
            if tokens[0]=="turn" or tokens[0]=='t':
                gprint("Turning {} Degrees".format(float(tokens[1])))
                angle=np.radians(float(tokens[1]))
                self.sub_behavior=Turn(angle)
            elif tokens[0]=="forward" or tokens[0][0]=='f':
                gprint("Forward {} Meters".format(float(tokens[1])))
                amount=float(tokens[1])
                if amount>0:
                    self.sub_behavior=AbortIfUltrasonicTooClose(0.20,MoveAhead(amount))
                else:
                    self.sub_behavior=MoveAhead(amount)
            elif tokens[0]=="slew":
                gprint("Slew {} Meters".format(float(tokens[1])))
                amount=float(tokens[1])
                self.sub_behavior=Slew(amount)
            elif tokens[0]=="move":
                vector=[float(tokens[1]),float(tokens[2]),float(tokens[3])]
                gprint("Moving {}".format(vector))
                kwargs["broker"].publish({"timestamp": time.time(),"move_command": {"type": "complex","distance": vector}},"move_command")
            elif tokens[0]=="savemap":
                gprint("Saving Local Map")
                shared_objects=kwargs["shared_objects"]
                with shared_objects.locks["occupancy_map"]:
                    graphmap=shared_objects.objects["occupancy_map"]
                    graphmap.save_to_file("local_map.npy")
            elif tokens[0]=="calibration_motion":
                gprint("Beginning Calibration Dance")
                #self.sub_behavior=CalibrateMotionBehavior()
                self.sub_behavior=RandomMotionTrackVisual()
            elif tokens[0]=="magnet":
                gprint_low("Magnetometer: {}".format(kwargs["short_term_memory"]["magnetometer/b_field"]["magnetometer/b_field"]))
            elif tokens[0]=="explore":
                self.sub_behavior=ExploreBehavior()
                ...
            elif tokens[0]=="goto":
                pass
            elif tokens[0]=="track":
                self.sub_behavior=TrackIfSeenBehavior("sports ball")
            elif tokens[0]=="pose" or tokens[0][0]=='p':
                pose=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose"]["latest_pose"])
                vel=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose_vel"]["latest_pose_vel"])
                dist=kwargs["short_term_memory"]["ultrasonic_sensor/last_measurement"]["ultrasonic_sensor/last_measurement"]["average_distance"]
                gprint_low("###Pose Report###")
                gprint_low("Pose: {}".format(pose.pretty_str()))
                gprint_low("Velocity: {}".format(vel.pretty_str()))
                gprint_low("Notes: {}".format(kwargs["short_term_memory"]["latest_pose"]["pose_notes"]))
                gprint_low("Ultrasonic Distance: {} m".format(dist))
                gprint_low("################")
            elif tokens[0]=="message" or tokens[0]=='m':
                key=tokens[1]
                value=tokens[2]
                kwargs["broker"].publish({"timestamp": time.time(),key: value},[key])
            elif tokens[0]=="calibrate":
                self.sub_behavior=RandomMotionTrackVisual()
            else:
                gprint("Could not process command {}".format(command))
        except Exception as e:
            gprint("Error processing command {}: {}".format(command,e))
            gprint("{}".format(traceback.format_exc()))

    def act(self,**kwargs):
        #gprint("act called")
        #broker=kwargs["broker"]
        textinput=kwargs["text_input"]
        try:
            text=textinput.get_nowait()
            self.commands.append(text)
            #gprint("got text {}".format(text))
        except:
            #gprint("{}".format(sys.exc_info()))
            pass #its empty, this is fine

        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
            if step_status==GratbotBehaviorStatus.COMPLETED:
                gprint_low("Completed Action")
                self.sub_behavior=None
            if step_status==GratbotBehaviorStatus.FAILED:
                gprint_low("Failed Action")
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
