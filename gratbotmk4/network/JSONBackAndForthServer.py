#import socketserver
import socket
import json
import threading,queue
import logging
import sys, time
import select
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
#logger.setLevel(logging.INFO)

#logging.basicConfig(level=logging.DEBUG)

class JSONBackAndForth():

    def __init__(self,debug=False,maxsize=10):
        self.debug=debug
        self.sock=None
        self.server_sock=None
        self.should_quit = False
        self.input_queue=queue.Queue(maxsize=maxsize)
        self.output_queue=queue.Queue(maxsize=maxsize)

    def start_client(self,host,port):
        self.host=host
        self.port=port
        #connect to server
        logger.info("creating client socket")
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.settimeout(5)
        logger.info("connecting")
        conninfo= self.sock.connect((self.host,self.port))

        logger.info("starting thread")

        self.thread = threading.Thread(target=self._thread_loop)
        self.thread.daemon = True
        self.thread.start()

    def join(self):
        self.should_quit=True
        self.thread.join()

    def start_server(self,port):
        logger.info("creating server socket")
        self.host = "10.0.0.4"
        self.port=port
        self.server_sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        #self.server_sock=socket.create_server( (self.host,self.port), reuse_port=True )
        self.server_sock.settimeout(5)
        logger.info("binding to {} {}".format(self.host,port))
        self.server_sock.bind((self.host, port))

        logger.info("listening")
        self.server_sock.listen(1) #accept only one connection at a time
        logger.info("thread starting")
        self.thread = threading.Thread(target=self._thread_loop_server)
        self.thread.daemon = True
        self.thread.start()

    def _thread_loop_server(self):
        while not self.should_quit:
            logger.debug("accepting")
            try:
                (self.sock, address) = self.server_sock.accept()
                self._thread_loop()
            except socket.timeout:
                ... #this is fine, just retry
            except Exception as e:
                logger.warning("unhandled exception in accept, closing connection")
                logger.warning("{}".format(e))

                #self.should_quit=True
        self.server_sock.close()


    def _thread_loop(self):
        with self.sock:
            #Should I do this?  Clear the output queue at the beginning
            #of connection in case stuff built up in the interim
            while not self.output_queue.empty():
                self.output_queue.get(block=False)
            while not self.should_quit:
                inputs=[self.sock]
                outputs=[self.sock]
                logger.debug("select")
                readable, writable, exceptional = select.select(inputs, outputs, inputs,5) #timeout 5 seconds
                logger.debug("select released")
                if self.sock in exceptional:
                    logger.error("Exception in socket")
                if self.sock in readable:
                    #self.data = self.rfile.readline().strip()
                    #first receive number of bytes message is
                    logger.debug("reading")

                    #data=self.sock.recv(1024)
                    length=self.sock.recv(4)
                    #logger.debug("got data {}".format(data))
                    if data==b'':
                        logger.info("Closing connection to ".format(self.host))
                        break
                    read_size=int.from_bytes(length)
                    logger.debug("readed length {}".format(length))
                    data=self.sock.recv(read_size)

                    #while data[-1]!=10:
                    #    logger.debug("data end character is {}".format(int(data[-1])))
                    #    newdata=self.sock.recv(1024)
                    #    data+=newdata
                    #    #logger.debug("got data {}".format(data))
                    #    if newdata==b'':
                    #        logger.warning("Connection broken ".format(self.host))
                    #        break

                    try:
                        #json_strings=data.decode().split('\n') #andle multiple messages all in one go
                        json_strings=data.decode() #andle multiple messages all in one go
                        json_strings.pop(-1)
                        for s in json_strings:
                            #logger.debug("message is {}".format(s))
                            datastructure=json.loads(s+'\n')
                            self.input_queue.put(datastructure)
                    except Exception as error:
                        logger.error("Error parsing json")
                        logger.exception(error)
                if self.sock in writable:
                    if not self.output_queue.empty():
                        logger.debug("writing")
                        #self.sock.sendall((json.dumps(self.output_queue.get())+"\n").encode())
                        tosend=(json.dumps(self.output_queue.get())).encode()
                        length=len(tosend).to_bytes(4)
                        self.sock.sendall(length)
                        self.sock.sendall(tosend)
                    else:
                        logger.debug("short sleep")
                        time.sleep(0.001) #just for a break
                logger.debug("loop end")
            logger.warning("closing socket")
            self.sock.close()

if __name__ == '__main__':
    #logger.basicConfig(level=logging.WARNING)
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
            logger.error("Keyboard interrupt")
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
            logger.error("Keyboard interrupt")
            client.join()
    else:
        print("don't know how to {}".format(sys.argv[1]))
