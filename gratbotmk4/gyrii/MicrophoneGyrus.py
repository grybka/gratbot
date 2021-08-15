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
        start_time=time.time()
        try:
            text=recognizer.recognize_sphinx(audio)
            logger.debug("Decoded audio as {}".format(text))
        except sr.UnknownValueError:
            logger.debug("Unintelligiable")
        except sr.RequestError as e:
            logger.debug("Sphinx error; {0}".format(e))
        logger.debug("It took {} ms to make the inference".format(1000*(time.time()-start_time)))
        #TODO handle this

    def start_thread_called(self):
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        self.stop_listening = self.r.listen_in_background(self.m, self.mic_callback)

    def join_called(self):
        if self.stop_listening is not None:
            self.stop_listening(wait_for_stop=False)
