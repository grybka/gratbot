
from Gyrus import ThreadedGyrus
import logging,time
import cv2
import numpy as np
import array
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class SoundDisplayGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.display=display
        super().__init__(broker)

    def get_keys(self):
        return ["microphone_data"]

    def get_name(self):
        return "SoundDisplayGyrus"

    def read_message(self,message):
        if "microphone_data" in message:
            data=message["microphone_data"]
            data = array.array('h', data)
            ps = np.abs(np.fft.fft(data))**2
            ps=ps[0:int(len(ps)/2)]
            yscale=np.max(ps)+1
            #time_step=1/16000
            #freqs = np.fft.fftfreq(len(data), time_step)
            image=np.zeros([200,320,3],dtype=np.uint8)
            for i in range(len(ps)):
                xpix=int(320*i/len(ps))
                ypix=199-int(200*ps[i]/yscale)
                image[ypix,xpix,:]=np.array([255,255,255])
            self.display.update_image("Microphone",image)