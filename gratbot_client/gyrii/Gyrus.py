
_all_gyrii={}

class Gyrus:
    def __init__(self):
        pass

    def save_config(self):
        return {}

    def load_config(self,config):
        pass

    def read_message(self,message):
        #Message is a dict object
        #returns a list of messages to put onto the queue
        return []

import threading,queue

class GyrusSharedObjects:
    def __init__(self):
        self.locks={}
        self.objects={}

    def add_object(self,object_name,value):
        self.locks[object_name]=threading.Lock()
        self.objects[object_name]=value

class ThreadedGyrus:
    def __init__(self,broker=None,shared_objects=None):
        self.broker=broker
        self.shared_objects=shared_objects
        if self.broker is not None:
            self.broker.subscribe(self.get_keys(),self.get_name())
            self.thread = threading.Thread(target=self._thread_loop)
            self.should_quit = False
            self.thread.daemon = True

    def start_thread(self):
        self.thread.start()

    def save_config(self):
        return {}

    def load_config(self,config):
        pass

    def read_message(self,message):
        #Message is a dict object
        pass

    def on_end(self):
        pass

    def get_keys(self):
        #override this.  Returns all keys that should be routed to my queue
        return []

    def get_name(self):
        #override this with the name of the gyrus
        return "Generic"

    def join(self):
        return self.thread.join()

    def _thread_loop(self):
        while not self.should_quit:
            try:
                message=self.broker.receive(self.get_name())
                self.read_message(message)
            except queue.Empty:
                pass
        self.on_end()

#template for a class where I can show images
class VideoDisplay:
    def update_image(self,windowname,image):
        pass
