import logging
import socket
import json

class GratbotClient:
    def __init__(self,host,port):
        self.host=host
        self.port=port
        self.sock=None

    def connect(self):
        #connect to server
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        return self.sock.connect((self.host,self.port))
    
    def disconnect(self):
        if not (self.sock is None):
            self.sock.close()
            self.sock=None

    def send_message(self,address,command,arguments):
        message={"address": address,"command": command,"arguments": arguments}
#logging.info("sending {}".format(json.dumps(message)))
        self.sock.sendall((json.dumps(message)+"\n").encode())
        data=self.sock.recv(1024)
#logging.info("received: {}".format(data))
        while len(data)==0 or data[-1]!=10:
#            if len(data)>0:
#logging.info("received: {}".format(data))
#logging.info("last elem |{}|".format(data[-1]))
            data+=self.sock.recv(1024)
#logging.info("received: {}".format(data))
        return json.loads(data)
 
    def __del__(self):
        self.disconnect()


