#import socketserver
import socket
import threading,queue
import logging
import sys, time

class JSONBackAndForth():

    def __init__(self,debug=False):
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
        if debug:
            print("creating client socket")
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        if debug:
            print("connecting")
        conninfo= self.sock.connect((self.host,self.port))
        self.thread.start()

    def start_server(self,port):
        if debug:
            print("creating server")
        self.sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port=port
        self.host = socket.gethostname()
        if debug:
            print("binding")
        self.sock.bind((self.host, port))
        if debug:
            print("listening")
        self.sock.listen(1) #accept only one connection at a time
        if debug:
            print("accepting")
        (clientsocket, address) = self.sock.accept()
        self.thread.start()

    def _thread_loop(self):
        while not self.should_quit:
            inputs=[self.sock]
            outputs=[self.sock]
            readable, writable, exceptional = select.select(inputs, outputs, inputs)
            if self.request in exceptional:
                logging.error("Exception in socket")
            if self.request in readable:
                #self.data = self.rfile.readline().strip()
                #first receive number of bytes message is
                header=self.sock.recv(8)
                length=int.from_bytes(header)
                counter=0
                buf=b''
                while(counter<length):
                    buf+=self.sock.recv(length-counter)
                if not self.data:
                    logging.info("Closing connection to ".format(self.client_address[0]))
                    break
                try:
                    datastructure=json.loads(buf.decode('utf-8'))
                    self.input_queue.put(datastructure)
                except Exception as error:
                    logging.error("Error parsing json")
                    logging.exception(error)
            if self.request in writable and not self.output_queue.empty():
                tosend=bytes(json.dumps(self.output_queue.get()))
                header=len(tosend).to_bytes(8)
                self.sock.send(header)
                self.sock.send(tosend)
                #self.wfile.write(bytes(json.dumps(self.output_queue.get())+"\n",encoding='utf-8'))

if __name__ == '__main__':
    if sys.argv[1]=='server':
        server=JSONBackAndForth()
        print("starting server")
        server.start_server(23033)
        last_send=0
        while True:
            time.sleep(0.001)
            if time.time()>last_send+1:
                server.output_queue.put({"body": "from server"})
            if not server.input_queue.empty():
                print("message receieved: ".format(server.input_queue.get()))
    elif sys.argv[1]=='client':
        client=JSONBackAndForth()
        client.start_client('localhost',20033)
        last_send=0
        while True:
            time.sleep(0.001)
            if time.time()>last_send+1.15:
                client.output_queue.put({"body": "from client"})
            if not client.input_queue.empty():
                print("message receieved: ".format(client.input_queue.get()))
    else:
        print("don't know how to {}".format(sys.argv[1]))
