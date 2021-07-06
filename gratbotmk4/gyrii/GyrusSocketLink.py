from Gyrus import ThreadedGyrus
import numpy as np
import socketserver
import threading,queue

#listen for requests.

class SocketGyrusLink(ThreadedGyrus):
    def __init__(self):
        self.keys=[]
        self.incoming_lock=threading.Lock()
        self.incoming=[]
        self.outgoing_lock=threading.Lock()
        self.outgoing=[]

    def get_keys(self):
        return self.keys

    def get_name(self):
        return "SocketGyrusLinkServer"

    def read_message(self,message):

class JSONBackAndForthServer(socketserver.StreamRequestHandler):
    gyruslink=None

    def handle(self):
        while True: #TODO I should probably disconnect after timeout
            self.data = self.rfile.readline().strip()
            if not self.data:
                logging.info("Closing connection to ".format(self.client_address[0]))
                break
            try:
                #receive all the incoming messages here
                datastructure=json.loads(self.data.decode('utf-8'))
                with self.gyrus_link.incoming_lock:
                    for obj in datastructure:
                        self.gyrus_link.incoming.append(obj)
            except Exception as error:
                logging.exception(error)
            #limit speed to 100 hz
            time.sleep(0.01)
            #send all the pending data here
            try:
                with self.gyrus_link.outgoing_lock:
                    self.wfile.write(bytes(json.dumps(self.gyrus_link.outgoing)+"\n",encoding='utf-8'))
                    self.gyrus_link.outgoing=[]
            except Exception as error:
                logging.exception(error)

class JSONBackAndForthClient()
