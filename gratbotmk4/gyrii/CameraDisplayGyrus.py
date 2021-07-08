
from Gyrus import ThreadedGyrus
import logging

class CameraDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display
        super().__init__(broker)

    def get_keys(self):
        return ["image"]

    def get_name(self):
        return "CameraDisplayGyrus"

    def read_message(self,message):
        self.display.update_image("camera",message["image"])
