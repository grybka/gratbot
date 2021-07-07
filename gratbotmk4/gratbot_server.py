#Gratbot server
sys.path.append('gyrii')
sys.path.append('network')
import logging
from JSONBackAndForthServer import JSONBackAndForthServer
from MessageBroker import MessageBroker

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)


#This stores the message passing
broker=MessageBroker()

#Open up a server to talk
test_port=23033
network_server=JSONBackAndForthServer()
network_server.start_server(test_port)

gyrii=GyrusList()
gyrii.append(SocketGyrusLink(broker,network_server.incoming_queue,network_server.outgoing_queue,[])) #TODO define keys here
gyrii.append(OakDGyrus(broker))


def main():
    try:
        #load gyrus configuration
        config_filename="config/server_gyrus_config.yaml"
        gyrii.load_and_start(config_filename)
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
