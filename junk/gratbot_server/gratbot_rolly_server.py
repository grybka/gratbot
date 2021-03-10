#A server that allows the bot to be controlled over tcp
import socketserver
import yaml
import time
import logging
import sys
import json

sys.path.append('../hardware_interface')
import hardware
import camera_interface

root = logging.getLogger()
#root.setLevel(logging.INFO)
root.setLevel(logging.DEBUG)

#message format
#in json, all on one line
#{ address: ["subsystem","endpoint"], command: "SET", arguments: "blah" }


class GratbotServer(socketserver.StreamRequestHandler):
    robot = None

    def handle(self):
        while True: #TODO I should probably disconnect after timeout
            self.data = self.rfile.readline().strip()
            if not self.data:
                logging.info("Closing connection to ".format(self.client_address[0]))
                break
        #print("{} wrote:".format(self.client_address[0]))
            try:
                logging.debug("received {}".format(self.data))
                datastructure=json.loads(self.data.decode('utf-8'))
                logging.debug("interpered as {}".format(json.dumps(datastructure)))
                if datastructure["command"]=="SET":
                    self.robot[datastructure["address"][0]].set(datastructure["address"][1],datastructure["arguments"])
                    self.wfile.write(bytes(json.dumps({ "response": "OK"})+"\n",encoding='utf-8'))
                elif datastructure["command"]=="GET":
                    ret=self.robot[datastructure["address"][0]].get(datastructure["address"][1])
                    self.wfile.write(bytes(json.dumps({ "response": ret})+"\n",encoding='utf-8'))
                else:
                    raise Exception("initial token not set or get")
            except Exception as error:
                logging.exception(error)
                error_response={}
                error_response["error"]="{}".format(error)
                self.wfile.write((json.dumps(error_response)+"\n").encode())
                #self.wfile.write(yaml.safe_dump({ "error": error}))

if __name__ == "__main__":
    #HOST, PORT = "localhost", 9999
    HOST, PORT = "10.0.0.4", 9999

    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    GratbotServer.robot=hardware.create_hardware(config_data["hardware"])
    robot_thread=hardware.GratbotHardwareThread(GratbotServer.robot)

    # Create the server, binding to localhost on port 9999
    server=None
    try:
        #with socketserver.TCPServer((HOST, PORT), GratbotServer) as server:
        server=socketserver.TCPServer((HOST, PORT), GratbotServer)
        logging.info("starting server")
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
        logging.error("this should be unreachable")
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
    finally:
        # return robot to safe state
        for x in GratbotServer.robot:
            GratbotServer.robot[x].shut_down()
        if server is not None:
            server.server_close()
