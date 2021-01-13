#import enum
from enum import Enum
import time
import sys

class GratbotBehaviorStatus(Enum):
    INPROGRESS = 1
    COMPLETED = 2
    FAILED = 3


class GratbotBehavior:
    def __init__(self):
        pass

    def act(self,comms,sensors):
        #Given comms and fused sensors, do something
        #return status,reserved
        #status is gratbotbehaviorstatus above (inprogress will be called again and ignore the rest)
        return GratbotBehaviorStatus.FAILED

    def reset(self):
        #in case I want to call this twice, reset to as if it had just been started
        pass

class GratbotBehavior_Series(GratbotBehavior):
    #Do a bunch of steps in order, abort on fail
    def __init__(self,step_array):
        super().__init__()
        self.step_array=step_array
        self.on_step=0
        self.should_loop=False

    def act(self,comms,sensors):
        if self.on_step>=len(self.step_array): #just in case its empty, really
            return GratbotBehaviorStatus.COMPLETED
        step_status=self.step_array[self.on_step].act(comms,sensors)
        if step_status==GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED
        if step_status==GratbotBehaviorStatus.COMPLETED:
            self.on_step+=1
            if self.on_step>=len(self.step_array): #just in case its empty, really
                if self.should_loop:
                    self.on_step=0
                    for i in range(len(self.step_array)):
                        self.step_array[i].reset()
                    return GratbotBehaviorStatus.INPROGRESS
                else:
                    return GratbotBehaviorStatus.COMPLETED
        return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.on_step=0
        for i in range(len(self.step_array)):
            self.step_array[i].reset()

class GratbotBehavior_Loop(GratbotBehavior):
    def __init__(self,sub_behavior,ntimes):
        self.sub_behavior=sub_behavior
        self.count=0

    def act(self,comms,sensors):
        if self.count>=ntimes:
            return GratbotBehaviorStatus.COMPLETED
        ret=self.sub_behavior.act(comms,sensors)
        if ret==GratbotBehaviorStatus.COMPLETED:
            self.sub_behavior.reset()
            self.count+=1
            if self.count>=ntimes:
                return GratbotBehaviorStatus.COMPLETED
            else:
                return GratbotBehaviorStatus.INPROGRESS
        if ret==GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED
        return ret

    def reset(self):
        self.sub_behavior.reset()
        self.count=0

class GratbotBehavior_Wait(GratbotBehavior):
    #just wait for a fixed amount of time
    def __init__(self,wait_time):
        super().__init__()
        self.wait_time=wait_time
        self.start_time=0

    def act(self,comms,sensors):
        if self.start_time==0:
            self.start_time=time.time()
        if time.time()>=(self.start_time+self.wait_time):
            return GratbotBehaviorStatus.COMPLETED
        return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.start_time=0

class GratbotBehavior_RecordSensorToMemory(GratbotBehavior):
    def __init__(self,sensor_address,memory_address):
        super().__init__()
        self.sensor_address=sensor_address
        self.memory_address=memory_address

    def act(self,comms,sensors):
        try:
            sensors.short_term_memory[self.memory_address]=sensors.gratbot_state[self.sensor_address]
        except Exception as e:
            print("Exception: {}".format(e))
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            traceback.print_exc(file=sys.stdout)
            return GratbotBehaviorStatus.FAILED
        return GratbotBehaviorStatus.COMPLETED
