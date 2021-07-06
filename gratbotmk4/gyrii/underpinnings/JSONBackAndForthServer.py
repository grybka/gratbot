#import socketserver
import socket
import json
import threading,queue
import logging
import sys, time
import select

class JSONBackAndForth():

    def __init__(self,debug=False):
        self.debug=debug
        self.sock=None
        self.thread = threading.Thread(target=self._thread_loop)
        self.should_quit = False
        self.thread.daemon = True
        self.input_queue=queue.Queue()
        self.output_queue=queue.Queue()

    def start_client(self,host,port):
        self.host=host
        self.port=port
        #connect to server
        if self.debug:
            print("creating client socket")
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        if self.debug:
            print("connecting")
        conninfo= self.sock.connect((self.host,self.port))
        self.thread.start()

    def start_server(self,port):
        if self.debug:
            print("creating server")
        self.sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port=port
        self.host = socket.gethostname()
        self.host = "10.0.0.4"
        if self.debug:
            print("binding to {} {}".format(self.host,port))
        self.sock.bind((self.host, port))
        if self.debug:
            print("listening")
        self.sock.listen(1) #accept only one connection at a time
        if self.debug:
            print("accepting")
        (clientsocket, address) = self.sock.accept()
        if self.debug:
            print("thread starting")
        self.thread.start()

    def _thread_loop(self):
        while not self.should_quit:
            inputs=[self.sock]
            outputs=[self.sock]
            print("calling select")
            readable, writable, exceptional = select.select(inputs, outputs, inputs)
            print("select released")
            print("lenngtsh from select {},{},{}".format(len(readable),len(writable),len(exceptional)))
            if self.sock in exceptional:
                logging.error("Exception in socket")
            if self.sock in readable:
                #self.data = self.rfile.readline().strip()
                #first receive number of bytes message is
                if self.debug:
                    print("reading")

                header=self.sock.recv(8)

                length=int.from_bytes(header,byteorder='big')
                if self.debug:
                    print("heading length is {}".format(length))

                counter=0
                buf=b''
                while(counter<length):
                    buf+=self.sock.recv(length-counter)
                if buf==b'':
                    logging.info("Closing connection to ".format(self.host))
                    break
                try:
                    datastructure=json.loads(buf.decode('utf-8'))
                    self.input_queue.put(datastructure)
                except Exception as error:
                    logging.error("Error parsing json")
                    logging.exception(error)
            if self.sock in writable: 
                if not self.output_queue.empty():
                    tosend=bytes(json.dumps(self.output_queue.get())).encode()
                    header=len(tosend).to_bytes(8,byteorder='big').encode()
                    if self.debug:
                        print("writing {} and {}".format(header,tosend))
                    self.sock.send(header)
                    self.sock.send(tosend)
                    #self.wfile.write(bytes(json.dumps(self.output_queue.get())+"\n",encoding='utf-8'))
                else:
                    print("I could send, but have nothing to say")

if __name__ == '__main__':
    test_port=23032
    if sys.argv[1]=='server':
        server=JSONBackAndForth(debug=True)
        print("starting server")
        server.start_server(test_port)
        print("server started")
        last_send=0
        while True:
            time.sleep(0.001)
            if time.time()>last_send+1:
                server.output_queue.put({"body": "from server"})
                last_send=time.time()
                #print("server sends")
            if not server.input_queue.empty():
                print("message receieved: ".format(server.input_queue.get()))
    elif sys.argv[1]=='client':
        client=JSONBackAndForth()
        #client.start_client('localhost',23033)
        client.start_client("10.0.0.4",test_port)
        print("client started")
        last_send=0
        while True:
            time.sleep(0.001)
            if time.time()>last_send+1.15:
                client.output_queue.put({"body": "from client"})
                last_send=time.time()
                #print("client sends")
            if not client.input_queue.empty():
                print("message receieved: ".format(client.input_queue.get()))
    else:
        print("don't know how to {}".format(sys.argv[1]))
