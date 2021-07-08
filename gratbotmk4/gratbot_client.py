#client to connect to server
import sys,os,traceback,time,threading
sys.path.append('gyrii')
sys.path.append('network')
import logging
import cv2 as cv
from network.JSONBackAndForthServer import JSONBackAndForth
from MessageBroker import MessageBroker
from gyrii.Gyrus import GyrusList,VideoDisplay
from gyrii.SocketGyrusLink import SocketGyrusLink
from gyrii.MessageLoggerGyrus import MessageLoggerGyrus
from gyrii.CameraDisplayGyrus import CameraDisplayGyrus
from gyrii.BehaviorGyrus import BehaviorGyrus
#from gyrii.behaviors.TextCommandBehavior import TextCommandBehavior
from gyrii.behaviors.CalibrateMotionBehavior import CalibrateMotionBehavior

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)

class DisplayLoop(VideoDisplay):
    def __init__(self):
        self.window_images={}
        self.open_windows=[]
        self.frame_lock=threading.Lock()

    def update_image(self,windowname,image):
        self.frame_lock.acquire()
        self.window_images[windowname]=image
        self.frame_lock.release()

    def one_loop(self):
        self.frame_lock.acquire()
        for key in self.window_images:
            if key not in self.open_windows:
                cv.namedWindow(key)
                self.open_windows.append(key)
            cv.imshow(key, self.window_images[key])
        self.frame_lock.release()
        key = cv.waitKey(30)

    def __del__(self):
        cv.destroyAllWindows()
display_loop=DisplayLoop()

class ClockLoop:
    def __init__(self,broker):
        self.broker=broker
        #self.clock_pulse_period=0.05
        self.clock_pulse_period=0.1
        self.keep_going=True
        self.frame_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while self.keep_going:
            time.sleep(self.clock_pulse_period)
            broker.publish({"timestamp": time.time(),"clock_pulse": self.clock_pulse_period},"clock_pulse")

    def stop(self):
        self.keep_going=False
        self.thread.join()

#This stores the message passing
broker=MessageBroker()

logging.debug("Starting Server")
test_port=23033
server_address="10.0.0.4"
network_client=JSONBackAndForth()
network_client.start_client(server_address,test_port)

logging.debug("Creating Gyrus List")
gyrii=GyrusList()
gyrii.append(MessageLoggerGyrus(broker,keys=["rotation_vector"]))
gyrii.append(SocketGyrusLink(broker,network_client.input_queue,network_client.output_queue,keys=["motor_command"]))
gyrii.append(CameraDisplayGyrus(broker,display_loop))
gyrii.append(BehaviorGyrus(broker,CalibrateMotionBehavior()))

def main():
    try:
        config_filename="config/client_gyrus_config.yaml"
        logging.debug("configuring and starting gyrii")
        gyrii.config_and_start(config_filename)
        logging.debug("gyrii started")
        clock_loop=ClockLoop(broker)
        logging.debug("clock started")
        while True:
            display_loop.one_loop()

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
