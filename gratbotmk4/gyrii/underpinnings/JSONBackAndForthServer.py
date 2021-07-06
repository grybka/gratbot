#import socketserver
import socket
import json
import threading,queue
import logging
import sys, time
import select
#logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.WARNING)

class JSONBackAndForth():

    def __init__(self,debug=False):
        self.debug=debug
        self.sock=None
        self.server_sock=None
        self.should_quit = False
        self.input_queue=queue.Queue()
        self.output_queue=queue.Queue()

    def start_client(self,host,port):
        self.host=host
        self.port=port
        #connect to server
        logging.debug("creating client socket")
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.settimeout(5)
        logging.debug("connecting")
        conninfo= self.sock.connect((self.host,self.port))
        logging.debug("starting thread")

        self.thread = threading.Thread(target=self._thread_loop)
        self.thread.daemon = True
        self.thread.start()

    def join(self):
        self.should_quit=True
        self.thread.join()

    def start_server(self,port):
        logging.debug("creating server socket")
        self.server_sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.settimeout(5)
        self.port=port
        self.host = socket.gethostname()
        self.host = "10.0.0.4"
        logging.debug("binding to {} {}".format(self.host,port))
        self.server_sock.bind((self.host, port))
        logging.debug("listening")
        self.server_sock.listen(1) #accept only one connection at a time
        logging.debug("thread starting")
        self.thread = threading.Thread(target=self._thread_loop_server)
        self.thread.daemon = True
        self.thread.start()

    def _thread_loop_server(self):
        while not self.should_quit:
            logging.debug("accepting")
            try:
                (self.sock, address) = self.server_sock.accept()
                self._thread_loop()
            except socket.timeout:
                ... #this is fine, just retry
            except:
                logging.warning("unhandled exception in accept, closing server")
                self.should_quit=True
        self.server_sock.close()


    def _thread_loop(self):
        with self.sock:
            while not self.should_quit:
                inputs=[self.sock]
                outputs=[self.sock]
                #logging.debug("select")
                readable, writable, exceptional = select.select(inputs, outputs, inputs,5) #timeout 5 seconds
                #logging.debug("select released")
                if self.sock in exceptional:
                    logging.error("Exception in socket")
                if self.sock in readable:
                    #self.data = self.rfile.readline().strip()
                    #first receive number of bytes message is
                    logging.debug("reading")

                    data=self.sock.recv(1024)
                    logging.debug("got data {}".format(data))
                    if data==b'':
                        logging.info("Closing connection to ".format(self.host))
                        break
                    while data[-1]!=10:
                        logging.debug("data end character is {}".format(int(data[-1])))
                        newdata=self.sock.recv(1024)
                        data+=newdata
                        logging.debug("got data {}".format(data))
                        if newdata==b'':
                            logging.warning("Connection broken ".format(self.host))
                            break

                    try:
                        json_strings=data.decode().split('\n') #andle multiple messages all in one go
                        json_strings.pop(-1)
                        for s in json_strings:
                            logging.debug("message is {}".format(s))
                            datastructure=json.loads(s+'\n')
                            self.input_queue.put(datastructure)
                    except Exception as error:
                        logging.error("Error parsing json")
                        logging.exception(error)
                if self.sock in writable: 
                    if not self.output_queue.empty():
                        logging.debug("writing")
                        self.sock.sendall((json.dumps(self.output_queue.get())+"\n").encode())
                    else:
                        time.sleep(0.001) #just for a break
            logging.warning("closing socket")
            self.sock.close()

if __name__ == '__main__':
    test_port=23033
    if sys.argv[1]=='server':
        server=JSONBackAndForth(debug=True)
        print("starting server")
        server.start_server(test_port)
        print("server started")
        last_send=0
        try:
            while True:
                time.sleep(0.01)
                if time.time()>last_send+1:
                    server.output_queue.put({"body": "from server"})
                    last_send=time.time()
                    #print("server sends")
                if not server.input_queue.empty():
                    print("message receieved: {} ".format(server.input_queue.get()))
        except KeyboardInterrupt:
            logging.error("Keyboard interrupt")
            server.join()

    elif sys.argv[1]=='client':
        try:
            client=JSONBackAndForth()
            #client.start_client('localhost',23033)
            client.start_client("10.0.0.4",test_port)
            print("client started")
            last_send=0
            while True:
                time.sleep(0.01)
                if time.time()>last_send+1.15:
                    client.output_queue.put({"body": "from client"})
                    last_send=time.time()
                    #print("client sends")
                if not client.input_queue.empty():
                    print("message receieved: {}".format(client.input_queue.get()))
        except KeyboardInterrupt:
            logging.error("Keyboard interrupt")
            client.join()
    else:
        print("don't know how to {}".format(sys.argv[1]))
