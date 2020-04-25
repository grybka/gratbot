#a class to keep track of the remote information about the hardware
import threading
import time
from GratbotClient import GratbotClient
import logging

class GratbotComms:
    def __init__(self,ip,port):
        self.client=GratbotClient(ip,port)
        self.client.connect()
        #TODO download info about the hardware

        self.comms_wait_period=0.1 #in seconds

        self.should_quit=False
        #store a bunch of belief data about the state of each thing
        self.intentions={} # intentions "SET" or "GET", arguments]
        self.intentions_lock=threading.Lock()
        self.hardware_state={}
        self.hardware_state_lock=threading.Lock()

        #start the thread
        self.should_quit=False
        self.thread=threading.Thread(target=self._loop)
        self.thread.daemon=True
        self.thread.start()

    def _loop(self):
        while not self.should_quit:
            time.sleep(self.comms_wait_period) #only try to message at 10 Hz
            self.intentions_lock.acquire()
            try:
                for endpoint in self.intentions:
                    splitpoint=endpoint.split('/')
                    address=splitpoint[:-1]
                    command=splitpoint[-1]
                    logging.debug("Sending: {} {} {}".format(address,command,self.intentions[endpoint]))
                    response=self.client.send_message(address,command,self.intentions[endpoint])
                    logging.debug("Response: {}".format(response))
                    if command=="GET":
                        self.hardware_state_lock.acquire()
                        self.hardware_state[endpoint]=response["response"]
                        self.hardware_state_lock.release()
            except Exception as err:
                logging.warning("Exception: {}".format(err))
            finally:
                self.intentions_lock.release()

    def set_intention(self,endpoint,value):
        if isinstance(endpoint,list):
            endpoint='/'.join(endpoint)
        self.intentions_lock.acquire()
        self.intentions[endpoint]=value
        self.intentions_lock.release()

    def get_state(self,endpoint):
        if isinstance(endpoint,list):
            endpoint='/'.join(endpoint)
        response={}
        self.hardware_state_lock.acquire()
        try:
            if endpoint in self.hardware_state:
                response=self.hardware_state[endpoint]
        except Exception as e:
            logging.warning("Exception in get state {}".format(e))
        finally:
            self.hardware_state_lock.release()
        return response

    def stop(self):
        self.should_quit=True
        if self.thread.is_alive():
            self.thread.join()
        self.client.disconnect()

    def __del__(self):
        self.client.disconnect()


