from enum import IntEnum
import time
import logging

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

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
        #second return value must be a dictionary
        return GratbotBehaviorStatus.FAILED,{}

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
            return GratbotBehaviorStatus.COMPLETED,{}
        return GratbotBehaviorStatus.INPROGRESS,{}

    def reset(self):
        self.start_time=0

class GratbotBehavior_Series(GratbotBehavior):
    #execute every step in the the series until a child
    #is found to either be in progress, or failure
    #return success if all childret return success
    def __init__(self,children):
        super().__init__()
        self.children=children
    def act(self,**kwargs):
        my_response_data={}
        for child in self.children:
            response,response_data=child.act(**kwargs)
            my_response_data.update(response_data)
            kwargs.update(response_data)
            if response==GratbotBehaviorStatus.FAILED:
                self.reset()
                return GratbotBehaviorStatus.FAILED,my_response_data
            if response==GratbotBehaviorStatus.INPROGRESS:
                return GratbotBehaviorStatus.INPROGRESS,my_response_data
        self.reset()
        return GratbotBehaviorStatus.COMPLETED,my_response_data

    def reset(self):
        for child in self.children:
            child.reset()

class GratbotBehavior_Checklist(GratbotBehavior):
    #like a series, but once a step has returned success, don't go bac kto it
    def __init__(self,children):
        super().__init__()
        self.children=children
        self.on_child=0
    def act(self,**kwargs):
        my_response_data={}
        while self.on_child<len(self.children):
            child=self.children[self.on_child]
            response,response_data=child.act(**kwargs)
            my_response_data.update(response_data)
            kwargs.update(response_data)
            if response==GratbotBehaviorStatus.FAILED:
                self.reset()
                return GratbotBehaviorStatus.FAILED,my_response_data
            if response==GratbotBehaviorStatus.INPROGRESS:
                return GratbotBehaviorStatus.INPROGRESS,my_response_data
            self.on_child+=1
        self.reset()
        return GratbotBehaviorStatus.COMPLETED,my_response_data

    def reset(self):
        for child in self.children:
            child.reset()
        self.on_child=0

class GratbotBehavior_Fallback(GratbotBehavior):
    #execute every step in the the series until a child
    #is found to either be in progress, or success
    #return failure if all childret return failure
    def __init__(self,children):
        super().__init__()
        self.children=children
    def act(self,**kwargs):
        my_response_data={}
        for child in self.children:
            response,response_data=child.act(**kwargs)
            my_response_data.update(response_data)
            if response==GratbotBehaviorStatus.COMPLETED:
                self.reset()
                return GratbotBehaviorStatus.COMPLETED,my_response_data
            if response==GratbotBehaviorStatus.INPROGRESS:
                return GratbotBehaviorStatus.INPROGRESS,my_response_data
        self.reset()
        return GratbotBehaviorStatus.FAILED,my_response_data

    def reset(self):
        for child in self.children:
            child.reset()

class GratbotBehavior_Choice(GratbotBehavior):
    def __init__(self,test_behavior,on_success=None,on_fail=None,on_inprogress=None):
        self.test_behavior=test_behavior
        self.on_success=on_success
        self.on_inprogress=on_inprogress

    def act(self,**kwargs):
        my_response_data={}
        response,response_data=self.test_behavior.act(**kwargs)
        my_response_data.update(response_data)
        if response==GratbotBehaviorStatus.COMPLETED:
            if self.on_success is not None:
                response,response_data=self.on_success.act(**kwargs)
                my_response_data.update(response_data)
                return response,my_response_data
            else:
                return GratbotBehaviorStatus.COMPLETED,my_response_data
        if response==GratbotBehaviorStatus.FAILED:
            if self.on_fail is not None:
                response,response_data=self.on_fail.act(**kwargs)
                my_response_data.update(response_data)
                return response,my_response_data
            else:
                return GratbotBehaviorStatus.FAILED,my_response_data
        if self.on_inprogress is not None:
            response,response_data=self.on_inprogress.act(**kwargs)
            my_response_data.update(response_data)
            return response,my_response_data
        return GratbotBehaviorStatus.INPROGRESS,my_response_data

class IgnoreFailure(GratbotBehavior):
    def __init__(self,child):
        self.child=child
    def act(self,**kwargs):
        response,response_data=self.child.act(**kwargs)
        if response==GratbotBehaviorStatus.INPROGRESS:
            return response, response_data
        return GratbotBehaviorStatus.COMPLETED, response_data



class DoOnce(GratbotBehavior):
    def __init__(self,child):
        self.child=child
        self.reset()
    def act(self,**kwargs):
        if self.is_complete:
            return GratbotBehaviorStatus.COMPLETED,self.response_data
        response,response_data=self.child.act(**kwargs)
        if response==GratbotBehaviorStatus.COMPLETED:
            self.response_data=response_data
            self.is_complete=True
        return response,response_data
    def reset(self):
        self.is_complete=False
        self.response_data={}


class TestElem(GratbotBehavior):
    def __init__(self,location,thetest,value):
        self.location=location
        self.value=value
        self.thetest=thetest

    def act(self,**kwargs):
        ondir=kwargs
        thetree=self.location
        if type(self.location) is not list:
            thetree=[ self.location ]
        for elem in thetree:
            if elem in ondir:
                ondir=ondir[elem]
        if self.thetest=='=':
            if ondir==self.value:
                return GratbotBehaviorStatus.COMPLETED,{}
        elif self.thetest=='<':
            if ondir<self.value:
                return GratbotBehaviorStatus.COMPLETED,{}
        elif self.thetest=='>':
            if ondir>self.value:
                return GratbotBehaviorStatus.COMPLETED,{}
        return GratbotBehaviorStatus.FAILED,{}


#class GratbotBehavior_Series(GratbotBehavior):
    #Do a bunch of steps in order, abort on fail
#    def __init__(self,step_array):
#        super().__init__()
#        self.step_array=step_array
#        self.on_step=0
#        self.should_loop=False
#
#    def act(self,**kwargs):
#        if self.on_step>=len(self.step_array): #just in case its empty, really
#            return GratbotBehaviorStatus.COMPLETED
#        step_status=self.step_array[self.on_step].act(**kwargs)
#        if step_status==GratbotBehaviorStatus.FAILED:
#            return GratbotBehaviorStatus.FAILED
#        if step_status==GratbotBehaviorStatus.COMPLETED:
#            self.on_step+=1
#            if self.on_step>=len(self.step_array): #just in case its empty, really
#                if self.should_loop:
#                    self.on_step=0
#                    for i in range(len(self.step_array)):
#                        self.step_array[i].reset()
#                    return GratbotBehaviorStatus.INPROGRESS
#                else:
#                    return GratbotBehaviorStatus.COMPLETED
#        return GratbotBehaviorStatus.INPROGRESS

#    def reset(self):
#        self.on_step=0
#        for i in range(len(self.step_array)):
#            self.step_array[i].reset()

#class GratbotBehavior_Loop(GratbotBehavior):
#    def __init__(self,sub_behavior,ntimes):
#        self.sub_behavior=sub_behavior
#        self.count=0
#        self.ntimes=ntimes
#
#    def act(self,**kwargs):
#        if self.count>=self.ntimes and self.ntimes!=-1:
#            return GratbotBehaviorStatus.COMPLETED
#        ret=self.sub_behavior.act(**kwargs)
#        if ret==GratbotBehaviorStatus.COMPLETED:
#            self.sub_behavior.reset()
#            self.count+=1
#            if self.count>=self.ntimes and self.ntimes!=-1:
#                return GratbotBehaviorStatus.COMPLETED
#            else:
#                return GratbotBehaviorStatus.INPROGRESS
#        if ret==GratbotBehaviorStatus.FAILED:
#            return GratbotBehaviorStatus.FAILED
#        return ret

#    def reset(self):
#        self.sub_behavior.reset()
#        self.count=0

class Announce(GratbotBehavior):
    def __init__(self,message,low=False):
        self.message=message
        self.low=low

    def act(self,**kwargs):
        if self.low:
            print(self.message)
        else:
            print(self.message)
        return GratbotBehaviorStatus.COMPLETED,{}
