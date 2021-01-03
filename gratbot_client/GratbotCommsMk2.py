#

import time
from GratbotClient import GratbotClient
import logging

class GratbotComms:
    def __init__(self,ip,port):
        self.client=GratbotClient(ip,port)
        self.client.connect()
        self.client_lock=threading.Lock()
        self.hardware_state_lock=threading.Lock()
        self.hardware_state={}
        self.last_hardware_state_update=0


    def immediate_get(self,endpoint):
        logging.debug("Getting: {}".format(endpoint))
        self.client_lock.acquire()
        response=self.client.send_message(endpoint,"GET",0)
        self.client_lock.release()
        logging.debug("Response: {}".format(response))
        return response["response"]

    def get_state(self,endpoint):
        if isinstance(endpoint,list):
            endpoint='/'.join(endpoint)
        response={}
        update_time=None
        self.hardware_state_lock.acquire()
        try:
            if endpoint in self.hardware_state:
                response=self.hardware_state[endpoint]
        except Exception as e:
            logging.warning("Exception in get state {}".format(e))
        finally:
            self.hardware_state_lock.release()
        return response

    def set(self,endpoint,value):
        logging.debug("Setting: {} to {}".format(endpoint,value))
        self.client_lock.acquire()
        response=self.client.send_message(endpoint,"GET",0)
        self.client_lock.release()
        logging.debug("Response: {}".format(response))
        return response["response"]

    def update(self):
        logging.debug("Requesting Status Update")
        response=self.client.send_message("","UPDATE",self.last_hardware_state_update)
        logging.debug("Response: {}".format(response))
        self.hardware_state_lock.acquire()
        for elem,value in response["response"].items():
            if elem=="timestamp":
                self.last_hardware_state_update=value
            self.hardware_state[elem]=value
        self.hardware_state_lock.release()
