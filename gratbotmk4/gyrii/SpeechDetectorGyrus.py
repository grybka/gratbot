
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
import wave
from collections import deque
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

class SpeechDetectorGyrus(ThreadedGyrus):
    def __init__(self,broker,save_to_file=False):
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
        #self.trigger_confidence=0.4  #0.25 looks too low.
        self.trigger_confidence=0.25  #0.25 looks too low.
        self.last_data=None
        self.is_recording=False
        self.min_sound_time=0.5
        self.records=[]
        self.last_record=b''
        self.save_to_file=save_to_file
        self.paudio=pyaudio.PyAudio()
        self.recent_confidences=deque([],maxlen=20)

    def get_keys(self):
        return ["microphone_data"]

    def get_name(self):
        return "SpeechDetectorGyrus"

    def end_and_save(self):
        self.is_recording=False
        if self.save_to_file:
            self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
            out_fname="sounds/sound_save_{}.wav".format(self.start_timestr)
            logger.debug("Saving {}".format(out_fname))
            wf = wave.open(out_fname, 'wb')
            wf.setnchannels(4)
            wf.setsampwidth(self.paudio.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(self.records))
            wf.close()
        #broadcast to broker
        self.broker.publish({"timestamp": time.time(),"speech_detected": self.records},["speech_detected"])
        self.last_record=b''
        self.records=[]

    def read_message(self,message):
        if "microphone_data" in message:
            audio_int16 = np.frombuffer(message["microphone_data"], np.int16);
            audio_int16=np.reshape(audio_int16,(-1,4))

            #select channel 1
            audio_float32 = torch.from_numpy(int2float(audio_int16[:,0]))
            with torch.no_grad():
                outs=self.model(audio_float32)
            confidences=outs[:,1]
            #logger.debug(confidences)
            if self.is_recording:
                self.records.append(message["microphone_data"])
                if (time.time()>self.recording_start+self.min_sound_time) and confidences[0]<self.trigger_confidence:
                    self.end_and_save()
            else:
                self.recent_confidences.append(confidences[0])
                if confidences[0]>self.trigger_confidence:
                    #logger.debug("start recording")
                    self.is_recording=True
                    self.recording_start=time.time()
                    self.records.append(self.last_record)
                    self.records.append(message["microphone_data"])
                #elif confidences[0]>0.1:
                #    logger.debug("confidence {}".format(confidences[0]))
            self.last_record=message["microphone_data"]
