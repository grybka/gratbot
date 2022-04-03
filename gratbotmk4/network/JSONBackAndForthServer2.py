
import socket
import json
import threading,queue
import logging
import sys, time
import select
logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
logger.setLevel(logging.INFO)

class JSONBackAndForth2:

    def __init__(self,debug=False,maxsize=10):
        self.debug=debug
        self.sock=None
        self.server_sock=None
        self.should_quit = False
        self.input_queue=queue.Queue(maxsize=maxsize)
        self.output_queue=queue.Queue(maxsize=maxsize)
        self.sender_thread=None
        self.recver_thread=None

    def join(self):
        self.should_quit=True
        if self.sender_thread is not None:
            self.sender_thread.join()
        if self.recver_thread is not None:
            self.recver_thread.join()
        if self.sock is not None:
            self.sock.close()
        if self.server_sock is not None:
            self.server_sock.close()

    def start_client(self,host,port):
        self.host=host
        self.port=port
        #connect to server
        logger.info("creating client socket")
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(5)
        logger.info("connecting")
        conninfo= self.sock.connect((self.host,self.port))

        logger.info("starting thread")

        self.sender_thread = threading.Thread(target=self._sender_thread_loop)
        self.sender_thread.daemon = True
        self.sender_thread.start()

        self.recver_thread = threading.Thread(target=self._recver_thread_loop)
        self.recver_thread.daemon = True
        self.recver_thread.start()

    def start_server(self,port):
        logger.info("creating server socket")
        self.host = "10.0.0.4"
        self.port=port
        self.server_sock= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
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

    def _sender_thread_loop(self):
        #Should I do this?  Clear the output queue at the beginning
        #of connection in case stuff built up in the interim
        while not self.output_queue.empty():
            self.output_queue.get(block=False)
        while not self.should_quit:
            to_send=[]
            try:
                to_send.append(self.output_queue.get(timeout=1))
                while not self.output_queue.empty():
                    to_send.append(self.output_queue.get_nowait())
                tosend=(json.dumps(to_send)).encode()
                length=len(tosend).to_bytes(4,byteorder='big')
                self.sock.sendall(length)
                self.sock.sendall(tosend)
            except queue.Empty:
                ...
            except socket.error as e:
                logger.error("sender socket error {}, closing".format(e))
                break


    def _thread_loop_server(self):
        while not self.should_quit:
            logger.debug("accepting")
            try:
                (self.sock, address) = self.server_sock.accept()

                self.sender_thread = threading.Thread(target=self._sender_thread_loop)
                self.sender_thread.daemon = True
                self.sender_thread.start()

                self.recver_thread = threading.Thread(target=self._recver_thread_loop)
                self.recver_thread.daemon = True
                self.recver_thread.start()

                self.sender_thread.join()
                self.recver_thread.join()
                self.sock.close()
                self.sock=None
            except socket.timeout:
                ... #this is fine, just retry
            except Exception as e:
                logger.warning("unhandled exception in accept, closing connection")
                logger.warning("{}".format(e))

                #self.should_quit=True
        self.server_sock.close()

    def _recver_thread_loop(self):
        while not self.should_quit:
            inputs=[self.sock]
            readable, writable, exceptional = select.select(inputs, [], inputs,5) #timeout 5 seconds
            if self.sock in exceptional:
                logger.error("Exception in socket")
            if self.sock in readable:
                try:
                    length=self.sock.recv(4)
                    if length==b'':
                        logger.info("Closing connection to ".format(self.host))
                        break
                    read_size=int.from_bytes(length,byteorder='big')
                    logger.debug("readed length {}".format(read_size))
                    data=b''
                    while(read_size>0):
                        newdata=self.sock.recv(read_size)
                        if newdata==b'':
                            logger.info("Closing connection to ".format(self.host))
                            break
                        data+=newdata
                        read_size=read_size-len(newdata)
                except Exception as error:
                    logger.error("Recvr error, closing")
                    break
                try:
                    #json_strings=data.decode().split('\n') #andle multiple messages all in one go
                    json_string=data.decode() #andle multiple messages all in one go
                    #json_strings.pop(-1)
                    #for s in json_strings:
                        #logger.debug("message is {}".format(s))
                    datastructure=json.loads(json_string+'\n')
                    #if "tracks" not in datastructure:
                    for elem in datastructure:
                        self.input_queue.put(elem)
                    #else:
                    #    logger.info("tracks discarded")
                except Exception as error:
                    logger.error("Error parsing json")
                    logger.exception(error)


if __name__ == '__main__':
    #logger.basicConfig(level=logging.WARNING)
    test_port=23033
    if sys.argv[1]=='server':
        server=JSONBackAndForth2(debug=True)
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
            client=JSONBackAndForth2()
            #client.start_client('localhost',test_port)
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
