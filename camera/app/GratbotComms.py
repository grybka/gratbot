#a class to keep track of the remote information about the hardware
import threading
from GratbotClient import GratbotClient

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
                    logging.debug("Sending: {} {} {}".format(endpoint[0],endpoint[1],self.intentions[endpoint])
                    response=self.client.send_message(endpoint[0],endpoint[1],self.intentions[endpoint])
                    logging.debug("Response: {}".format(response))
                    if endpoint[1]=="GET":
                        self.hardware_state_lock.acquire()
                        self.hardware_state[endpoint[0]]=response["response"]
                        self.hardware_state_lock.release()
            except Exception as err:
                logging.warning("Exception: {}".format(err))
            finally:
                self.intentions_lock.release()

    def set_intention(endpoint,value):
        self.intentions_lock.acquire()
        self.intentions[endpoint]=value
        self.intentions_lock.release()

    def stop(self):
        self.should_quit=True
        if self.thread.is_alive():
            self.thread.join()
        self.client.disconnect()

    def __del__(self):
        self.client.disconnect()


