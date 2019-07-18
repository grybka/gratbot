#A server that allows the bot to be controlled over tcp
import socketserver
import yaml
import time
import logging

sys.path.append('../hardware_interface')
import hardware

root = logging.getLogger()
root.setLevel(logging.INFO)


class GratbotServer(socketserver.StreamRequestHandler):
    robot = None

    def handle(self):
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        tokens=self.data.split(" ")
        try:
            if len(tokens)<1:
                raise Exception("only {} tokens found".format(len(tokens)))
            if tokens[0]=="GET":
                address=tokens[1].split(":")
                response=self.robot[address[0]].get(address[1])
                self.wfile.write(response)
            if tokens[0]=="SET":
                address=tokens[1].split(":")
                self.robot[address[0]].set(address[1],tokens[2])
                self.wfile.write("OK")
            else:
                #unhandeled thing
                raise Exception("initial token not set or get")
        except Exception as error:
            logger.error(error)
            self.wfile.write("ERROR")
#print(self.data)
        # Likewise, self.wfile is a file-like object used to write back
        # to the client
#self.wfile.write(self.data.upper())


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


