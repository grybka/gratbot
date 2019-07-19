#A server that allows the bot to be controlled over tcp
import socketserver
import yaml
import time
import logging

sys.path.append('../hardware_interface')
import hardware

root = logging.getLogger()
root.setLevel(logging.INFO)

#message format
#in json, all on one line
#{ address: ["subsystem","endpoint"], command: "SET", arguments: "blah" }


class GratbotServer(socketserver.StreamRequestHandler):
    robot = None

    def handle(self):
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        try:
            datastructure=yaml.safe_load(self.data)
            if datastructure["command"]=="SET":
                self.robot[datastructure["address"][0]].set(datastructure["address"][1],datastructure["arguments"])
                self.wfile.write(yaml.safe_dump({ "response": "OK"}))
            elif datastructure["command"]=="GET":
                ret=self.robot[datastructure["address"][0]].get(datastructure["address"][1])
                self.wfile.write(yaml.safe_dump({ "response": ret}))
            else:
                raise Exception("initial token not set or get")
        except Exception as error:
            logger.error(error)
            self.wfile.write(yaml.safe_dump({ "error": error}))

if __name__ == "__main__":
    HOST, PORT = "localhost", 9999

    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    GratbotServer.robot=hardware.create_hardware(config_data["hardware"])


    # Create the server, binding to localhost on port 9999
    try:
        with socketserver.TCPServer((HOST, PORT), GratbotServer) as server:
            # Activate the server; this will keep running until you
            # interrupt the program with Ctrl-C
            server.serve_forever()
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
    finally:
        #return robot to safe state
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)


