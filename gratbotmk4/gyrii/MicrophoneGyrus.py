from Gyrus import ThreadedGyrus
import speech_recognition as sr

import logging
import time
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class MicrophoneGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.m = sr.Microphone()
        self.r = sr.Recognizer()
        self.stop_listening=None

    def get_keys(self):
        return [""]

    def get_name(self):
        return "MicrophoneGyrus"

    def mic_callback(self,recognizer, audio):
        logger.debug("Mic callback called")
        #TODO handle this

    def start_thread_called(self):
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        self.stop_listening = self.r.listen_in_background(self.m, self.mic_callback)

    def join_called(self):
        if self.stop_listening is not None:
            self.stop_listening(wait_for_stop=False)
