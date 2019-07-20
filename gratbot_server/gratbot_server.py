#A server that allows the bot to be controlled over tcp
import SocketServer
import yaml
import time
import logging
import sys
import json

sys.path.append('../hardware_interface')
import hardware

root = logging.getLogger()
root.setLevel(logging.INFO)

#message format
#in json, all on one line
#{ address: ["subsystem","endpoint"], command: "SET", arguments: "blah" }


class GratbotServer(SocketServer.StreamRequestHandler):
    robot = None

    def handle(self):
        while True: #TODO I should probably disconnect after timeout
            self.data = self.rfile.readline().strip()
            if not self.data:
                logging.info("Closing connection to ".format(self.client_address[0]))
                break
        #print("{} wrote:".format(self.client_address[0]))
            try:
                logging.info("received {}".format(self.data))
                datastructure=json.loads(self.data)
                logging.info("interpered as {}".format(json.dumps(datastructure)))
                if datastructure["command"]=="SET":
                    self.robot[datastructure["address"][0]].set(datastructure["address"][1],datastructure["arguments"])
                    self.wfile.write(json.dumps({ "response": "OK"})+"\n")
                elif datastructure["command"]=="GET":
                    ret=self.robot[datastructure["address"][0]].get(datastructure["address"][1])
                    self.wfile.write(json.dumps({ "response": ret}))
                else:
                    raise Exception("initial token not set or get")
            except Exception as error:
                logging.error(error)
                error_response={}
                error_response["error"]="{}".format(error)
                self.wfile.write((json.dumps(error_response)+"\n").encode())
                #self.wfile.write(yaml.safe_dump({ "error": error}))

if __name__ == "__main__":
    #HOST, PORT = "localhost", 9999
    HOST, PORT = "10.0.0.5", 9999

    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    GratbotServer.robot=hardware.create_hardware(config_data["hardware"])


    # Create the server, binding to localhost on port 9999
    server=None
    try:
        #with SocketServer.TCPServer((HOST, PORT), GratbotServer) as server:
        server=SocketServer.TCPServer((HOST, PORT), GratbotServer)
        logging.info("starting server")
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
        logging.error("this should be unreachable")
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
    finally:
        #return robot to safe state
        if server is not None:
            server.server_close()
        GratbotServer.robot["wheel_motor"].stop()
        GratbotServer.robot["wheel_turn_servo"].setpos_fraction(0)


