from Gyrus import ThreadedGyrus
import time
import json

from MessageToJSON import message_to_json

class MessageLoggerGyrus(ThreadedGyrus):
    def __init__(self,broker,keys=[]):
        self.keys=keys
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        #self.save_filename = "logs/sensor_log_{}.txt".format(self.start_timestr)
        self.save_filename = "logs/sensor_log_latest.txt".format(self.start_timestr)
        self.f=open(self.save_filename,"w")
        super().__init__(broker)

    def on_end(self):
        self.f.close()

    def read_message(self,message):
        try:
            self.f.write(json.dumps(message_to_json(message))+"\n")
        except:
            print("Error writing message {}".format(message_to_json(message)))
        #f.close()
        return []

    def get_keys(self):
        return self.keys

    def get_name(self):
        return "MessageLogger"
