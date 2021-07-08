
from Gyrus import ThreadedGyrus
from behaviors.Behavior import GratbotBehaviorStatus
import time


class BehaviorGyrus(ThreadedGyrus):
    def __init__(self,broker,on_behavior,pass_kwargs={}):
        self.short_term_memory={}
        self.on_behavior=on_behavior
        self.my_kwargs=pass_kwargs
        self.skip_until_time=0
        super().__init__(broker)


    def get_keys(self):
        return ["clock_pulse"]

    def get_name(self):
        return "BehaviorGyrus"

    def read_message(self,message):
        #Short term memory
        for key in message:
            if key=="timestamp": #don't need to save that
                continue
            self.short_term_memory[key]=message
        if "clock_pulse" in message:
            if message["timestamp"]>self.skip_until_time:
                self.execute_behavior()

    def execute_behavior(self):
        #returns a list of messages
        if self.on_behavior==None:
            return []
        the_kwargs=self.my_kwargs
        the_kwargs["short_term_memory"]=self.short_term_memory
        the_kwargs["broker"]=self.broker
        #resp=self.on_behavior.act(short_term_memory=self.short_term_memory,message_queue=message_queue)
        resp=self.on_behavior.act(**the_kwargs)
        if resp==GratbotBehaviorStatus.COMPLETED:
            print("Behavior Completed.  Halting")
            on_behavior=None
        if resp==GratbotBehaviorStatus.COMPLETED:
            print("Behavior Failed.  Halting")
            on_behavior=None
        self.skip_until_time=time.time()
