
from Gyrus import ThreadedGyrus

import logging
import time
import pyaudio
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class MicrophoneGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.paudio=pyaudio.PyAudio()
        devinfo=self.paudio.get_default_input_device_info()
        logger.debug("Connected to audio input device {}".format(devinfo))
        self.sample_size=1 #in seconds
        self.format=pyaudio.paInt16
        self.chunk=4096
        #self.sample_rate=44100
        self.sample_rate=16000
        self.dev_index=1
        self.chans=1
        self.mic_thread=None


    def get_keys(self):
        return [""]

    def get_name(self):
        return "MicrophoneGyrus"

    def start_thread_called(self):
        self.mic_thread = threading.Thread(target=self._mic_thread_loop)
        self.mic_thread.daemon = True
        self.mic_thread.start()

    def join_called(self):
        if self.mic_thread is not None:
            return self.mic_thread.join()
        return None

    def _mic_thread_loop(self):
        self.stream=self.paudio.open(format=self.format,rate=self.sample_rate,channels=self.chans,input_device_index=self.dev_index,input=True,frames_per_buffer=self.chunk)
        #this thread stops motors when they are supposed to stop
        while not self.should_quit:
            rec=self.stream.read(self.chunk)
            self.broker.publish({"timestamp": time.time(),"microphone_data": rec},["microphone_data"])
        self.stream.stop_stream()
        self.stream.close()
