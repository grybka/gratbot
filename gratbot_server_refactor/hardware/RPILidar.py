
from GratbotSpimescape import GratbotSpimescape
from GratbotSpimescape import _all_gratbot_spimescapes
from adafruit_rplidar import RPLidar
import threading
import time

#datasheet notes
#distance range 0.15 to 12
#distance resolution max(0.5 mm,1% of range)
#angelar resolution 1 degree
class RPILidarA1(GratbotSpimescape):

    def __init__(self, datastruct, hardware):
        #maybe /dev/ttyUSB0?
        #self.lidar=RPLidar(None,datastruct["usb_port_name"])
        self.lidar=RPLidar(None,"/dev/ttyUSB0")
        print("Lidar may be initialized, how would I know?")
        #print("Lidar initiazised with info {}".format(self.lidar.get_info()))

        self.end_called=False
        self.last_scan=[] #array of quality,angle,distance
        self.last_update=0
        self.data_lock=threading.Lock()
        self.thread = threading.Thread(target=self._run_thread)
        self.thread.daemon = True
        self.thread.start()


    def _run_thread(self):
        while not self.end_called:
            try:
                for scan in self.lidar.iter_scans():
                    with self.data_lock:
                        self.last_scan=scan
                        self.last_update=time.time()
            finally:
                self.lidar.stop()
                self.lidar.disconnect()

    def set(self, endpoint, value):
        return None

    def get(self, endpoint):
        #this seems so fast that I should just do it all in update
        #if endpoint=="field":
        #    return self.sensor.magnetic
        raise Exception("unknown camera endpoint {}".format(endpoint))


    def get_update(self,last_time): #request changes since time as dictionary
        ret={}
        with self.data_lock:
            if self.last_update>last_time:
                ret["lidar_scan"]=self.last_scan.copy()
        return ret

    def shut_down(self):
        self.end_called=True

    def __del__(self):
        self.end_called=True
        self.thread.join()

_all_gratbot_spimescapes["Lidar"]=RPILidarA1
