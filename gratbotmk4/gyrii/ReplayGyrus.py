
from Gyrus import ThreadedGyrus
import logging
import json
import threading
import time
from MessageToJSON import json_to_message,message_to_json

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

#this takes a log file and replays the logs as if they were coming out of a socket gyrus

def load_sensor_log_file(fname):
    response=[]
    first_timestamp=0
    f=open(fname,'r')
    for line in f.readlines():
        dat=json_to_message(json.loads(line))
        if "timestamp" in dat:
            if first_timestamp==0:
                first_timestamp=dat["timestamp"]
            #timestamp=dat["timestamp"]-first_timestamp
        response.append(dat)
    return response,first_timestamp

class ReplayGyrus(ThreadedGyrus):
    def __init__(self,broker,log_file_name):
        self.receive_thread = threading.Thread(target=self._receive_thread_loop)
        self.receive_thread.daemon = True
        logger.info("loading replay file")
        self.data,self.first_timestamp=load_sensor_log_file(log_file_name)
        logger.info("replay file loaded")
        self.on_data_elem=0
        self.my_start_time=0
        self.slo_mo=2
        super().__init__(broker)

    def start_thread_called(self):
        self.receive_thread.start()

    def join_called(self):
        return self.receive_thread.join()

    def get_keys(self):
        return []

    def get_name(self):
        return "ReplayGyrus"

    def _receive_thread_loop(self):
        self.my_start_time=time.time()
        while not self.should_quit:
            #logger.info(" on sim elem {}".format(self.on_data_elem))
            #skip entries without a timestamp
            while self.on_data_elem<len(self.data) and "timestamp" not in self.data[self.on_data_elem]:
                logger.warning("No timestamp found in sim log elem")
                self.on_data_elem+=1
            #loop at end, make sure to reset my timestame
            if self.on_data_elem>=len(self.data):
                self.on_data_elem=0
                self.my_start_time=time.time()
            message=self.data[self.on_data_elem]
            thetime=self.slo_mo*(message["timestamp"]-self.first_timestamp)+self.my_start_time
            #logger.debug("message timestamp {}, first timestamp{} , mystart time {}".format(message["timestamp"],self.first_timestamp,self.my_start_time))
            #logger.debug("so emit message in: {} seconds".format(thetime-time.time()))
            if thetime<time.time():
                self.broker.publish(message,message["keys"])
                logger.debug(" emitting message {} with keys {}".format(self.on_data_elem,message["keys"]))
                self.on_data_elem+=1
            else:
                time.sleep(0.01)
