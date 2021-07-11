#Gratbot server
import sys,os,traceback,time
sys.path.append('gyrii')
sys.path.append('network')
import logging
from network.JSONBackAndForthServer import JSONBackAndForth
from MessageBroker import MessageBroker
from gyrii.Gyrus import GyrusList
from gyrii.SocketGyrusLink import SocketGyrusLink
from gyrii.MotorGyrus import MotorGyrus
from gyrii.MessageLoggerGyrus import MessageLoggerGyrus
from OakDGyrus import OakDGyrus

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)


#This stores the message passing
broker=MessageBroker()

#Open up a server to talk
logging.debug("Starting Server")
test_port=23033
network_server=JSONBackAndForth()
network_server.start_server(test_port)

logging.debug("Creating Gyrus List")
gyrii=GyrusList()
gyrii.append(SocketGyrusLink(broker,network_server.input_queue,network_server.output_queue,keys=["rotation_vector","image","detections","motor_response"])) #TODO define keys here
gyrii.append(OakDGyrus(broker))
gyrii.append(MotorGyrus(broker))
gyrii.append(MessageLoggerGyrus(broker,keys=["motor_command","motor_response"]))


def main():
    try:
        #load gyrus configuration
        config_filename="config/server_gyrus_config.yaml"
        logging.debug("configuring and starting gyrii")
        gyrii.config_and_start(config_filename)
        logging.debug("gyrii started")
        while True:
            time.sleep(1)
            ...
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended")
    except Exception as e:
        print("Exception: {}".format(e))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        traceback.print_exc(file=sys.stdout)
    finally:
        gyrii.quit_and_join()

main()
