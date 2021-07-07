from Gyrus import ThreadedGyrus
import numpy as np
import cv2
import threading,queue
import logging

#listen for requests.

#messages must have
#"keys"

def object_to_json(object):
    if type(object)==dict: #walk through dicts
        ret={}
        for key in object:
            ret[key]=object_to_json(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(object_to_json(elem))
        return ret
    if type(object)==np.ndarray:
        if object.dtype==np.uint8:
            #images are special
            _,encoded=cv2.imencode('.jpg',object)
            encoded=encoded.tobytes()
            x=base64.b64encode(encoded)
            return {"_packed_type":"jpegimage","data":x.decode('utf-8')}
        else:
            return {"_packed_type":"ndarray","data": object_to_json(object.tolist())}
    #presume anything else is OK.  I could have some more checks here
    if type(object) in [str,int,float,complex,bool]:
        return object
    raise Exception("object_to_json can't handle type: {}".format(type(object)))

def json_to_object(object):
    if type(object)==dict: #walk through dicts
        if "_packed_type" in object:
            if object["_packed_type"]=="jpegimage":
                x=bytes(object["data"],encoding='utf-8')
                x=base64.b64decode(x)
                x=np.frombuffer(x,dtype=np.uint8)
                return cv.imdecode(x,cv.IMREAD_COLOR)
            if object["_packed_type"]=="ndarray":
                return np.array(object["data"])
            else:
                raise Exception("json_to_object doesn't understand type {}".format(object["_packed_type"]))
        ret={}
        for key in object:
            ret[key]=json_to_object(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(json_to_object(elem))
    return object


class SocketGyrusLink(ThreadedGyrus):
    def __init__(self,broker,incoming_queue,outgoing_queue,keys=[]):
        self.keys=keys
        self.incoming_queue=incoming_queue
        self.outgoing_queue=outgoing_queue
        self.receive_thread = threading.Thread(target=self._receive_thread_loop)
        self.receive_thread.daemon = True
        super().__init__(broker)

    def start_thread_called(self):
        self.receive_thread.start()

    def join_called(self):
        return self.receive_thread.join()

    def get_keys(self):
        return self.keys

    def get_name(self):
        return "SocketGyrusLinkServer"

    def read_message(self,message):
        if self.outgoing_queue.full():
            self.outgoing_queue.get() #throw away packets if they aren't getting sent
        self.outgoing_queue.put(object_to_json(message),timeout=0.1)

    def _receive_thread_loop(self):
        while not self.should_quit:
            try:
                broadcast_message=self.incoming_queue.get(block=True,timeout=1)
                object_to_broadcast=json_to_object(broadcast_message)
                if "keys" in object_to_broadcast:
                    self.broker.publish(object_to_broadcast,object_to_broadcast["keys"])
                else:
                    logging.error(" keys not in socketgyruslink message! {}".format(broadcast_message))
            except queue.Empty:
                ... #no big deal, just try again
        logging.debug("SocketGyrusLink receive_thread quitting")
