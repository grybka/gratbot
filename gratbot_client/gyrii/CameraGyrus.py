
from GratbotUV4LConnection import GratbotUV4LConnection
from Gyrus import ThreadedGyrus
import time

class CameraGyrus(ThreadedGyrus):
    def __init__(self,broker,simulation_mode=False):
        self.simulation_mode=simulation_mode
        if not self.simulation_mode:
            self.video_connection=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")
        super().__init__(broker)

    def get_keys(self):
        return [ "clock_pulse","motor_command" ]

    def get_name(self):
        return "CommsUpdateGyrus"

    def read_message(self,message):
        if "clock_pulse" in message:
