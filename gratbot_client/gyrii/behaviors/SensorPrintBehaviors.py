from Behavior import GratbotBehavior
from Behavior import GratbotBehaviorStatus
import time
import cv2 as cv

class PrintState(GratbotBehavior):
    def __init__(self,state_name):
        self.state_name=state_name

    def act(self,**kwargs):
        if self.state_name not in kwargs["short_term_memory"]:
            print("Error, {} not in memory".format(self.state_name))
        else:
            print("{}: {}".format(self.state_name,kwargs["short_term_memory"][self.state_name][self.state_name]))
        return GratbotBehaviorStatus.COMPLETED

    def reset(self):
        #in case I want to call this twice, reset to as if it had just been started
        pass

class SaveFrame(GratbotBehavior):
    def __init__(self):
        pass

    def act(self,**kwargs):
        image_name="camera_frame"
        if image_name not in kwargs["short_term_memory"]:
            logger.error("No camera frame in short term memory, cannot log")
            return GratbotBehaviorStatus.FAILED
        else:
            frame=kwargs["short_term_memory"][image_name][image_name]
            timestamp=kwargs["short_term_memory"][image_name]["timestamp"]

            fname=time.strftime("images/image_%Y%m%d-%H%M%S.png")
            cv.imwrite(fname, frame)
            kwargs["message_queue"].append({"timestamp": timestamp,"saved_image": fname})
        return GratbotBehaviorStatus.COMPLETED
