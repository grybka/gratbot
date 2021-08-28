
from Gyrus import ThreadedGyrus
import threading

import logging
import time
import pyaudio
import io
import numpy as np
import torch
torch.set_num_threads(1)
import torchaudio
import matplotlib
import matplotlib.pylab as plt
#torchaudio.set_audio_backend("soundfile")
import pyaudio
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Provided by Alexander Veysov
def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1/abs_max
    sound = sound.squeeze()  # depends on the use case
    return sound

class SpeedDetectorGyrus(ThreadedGyrus):
    def __init__(self):
        super().__init__(broker)
        self.model, self.utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                              model='silero_vad',
                              force_reload=False)
        (get_speech_ts,
         get_speech_ts_adaptive,
         save_audio,
         read_audio,
         state_generator,
         single_audio_stream,
         collect_chunks) = self.utils

    def get_keys(self):
        return [""]

    def get_name(self):
        return "SpeechDetectorGyrus"

    def read_message(self,message):
        if "microphone_data" in message:
            audio_int16 = np.frombuffer(message["microphone_data"], np.int16);
            audio_float32 = torch.from_numpy(int2float(audio_int16))
            with torch.no_grad():
                outs=self.model(audio_float32)
            confdidences=outs[:,1]
            logger.debug("Confidences {}".format(confdidences))
