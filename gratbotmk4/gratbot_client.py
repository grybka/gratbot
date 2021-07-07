#client to connect to server
import sys,os,traceback,time
sys.path.append('gyrii')
sys.path.append('network')
import logging
from network.JSONBackAndForthServer import JSONBackAndForth
from MessageBroker import MessageBroker
from gyrii.Gyrus import GyrusList
from gyrii.SocketGyrusLink import SocketGyrusLink
from gyrii.MessageLoggerGyrus import MessageLoggerGyrus

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)

logging.debug("Starting Server")
test_port=23033
server_address="10.0.0.4"
network_client=JSONBackAndForth()
network_client.start_client("10.0.0.4",test_port)

logging.debug("Creating Gyrus List")
gyrii=GyrusList()
gyrii.append(MessageLoggerGyrus(broker,keys=["rotation_vector"]))
gyrii.append(SocketGyrusLink(broker,network_client.input_queue,network_server.client,keys=[]))

def main():
    try:
        config_filename="config/client_gyrus_config.yaml"
        logging.debug("configuring and starting gyrii")
        gyrii.config_and_start(config_filename)
        logging.debug("gyrii started")
        while True:
            time.sleep(1)

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
