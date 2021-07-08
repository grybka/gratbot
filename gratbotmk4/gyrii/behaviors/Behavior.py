from enum import IntEnum
import time

class GratbotBehaviorStatus(IntEnum):
    INPROGRESS = 1
    COMPLETED = 2
    FAILED = 3

class GratbotBehavior:
    def __init__(self):
        pass

    def act(self,**kwargs):
        #Given comms and fused sensors, do something
        #return status
        #status is gratbotbehaviorstatus above (inprogress will be called again and ignore the rest)
        return GratbotBehaviorStatus.FAILED

    def reset(self):
        #in case I want to call this twice, reset to as if it had just been started
        pass

class GratbotBehavior_Wait(GratbotBehavior):
    #just wait for a fixed amount of time
    def __init__(self,wait_time):
        super().__init__()
        self.wait_time=wait_time
        self.start_time=0

    def act(self,**kwargs):
        if self.start_time==0:
            self.start_time=time.time()
        if time.time()>=(self.start_time+self.wait_time):
            return GratbotBehaviorStatus.COMPLETED
        return GratbotBehaviorStatus.INPROGRESS

    def reset(self):
        self.start_time=0


class GratbotBehavior_Series(GratbotBehavior):
    #Do a bunch of steps in order, abort on fail
    def __init__(self,step_array):
        super().__init__()
        self.step_array=step_array
        self.on_step=0
        self.should_loop=False

    def act(self,**kwargs):
        if self.on_step>=len(self.step_array): #just in case its empty, really
            return GratbotBehaviorStatus.COMPLETED
        step_status=self.step_array[self.on_step].act(**kwargs)
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
        self.ntimes=ntimes

    def act(self,**kwargs):
        if self.count>=self.ntimes and self.ntimes!=-1:
            return GratbotBehaviorStatus.COMPLETED
        ret=self.sub_behavior.act(**kwargs)
        if ret==GratbotBehaviorStatus.COMPLETED:
            self.sub_behavior.reset()
            self.count+=1
            if self.count>=self.ntimes and self.ntimes!=-1:
                return GratbotBehaviorStatus.COMPLETED
            else:
                return GratbotBehaviorStatus.INPROGRESS
        if ret==GratbotBehaviorStatus.FAILED:
            return GratbotBehaviorStatus.FAILED
        return ret

    def reset(self):
        self.sub_behavior.reset()
        self.count=0

class Announce(GratbotBehavior):
    def __init__(self,message,low=False):
        self.message=message
        self.low=low

    def act(self,**kwargs):
        if self.low:
            print(self.message)
        else:
            print(self.message)
        return GratbotBehaviorStatus.COMPLETED
