

from Gyrus import ThreadedGyrus
import logging,time
import cv2
import numpy as np
import pyaudio
import wave
import array
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class SoundRecordGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.is_recording=False
        self.min_sound_time=1.0
        self.avg_energy=0
        self.avg_run=0.9
        self.records=[]
        self.paudio=pyaudio.PyAudio()


    def get_keys(self):
        return ["microphone_data","gyrus_config"]

    def get_name(self):
        return "SoundRecordGyrus"

    def get_chunk_energy(self,data):
        data = array.array('h', data)
        my_sum=0
        for i in range(len(data)):
            my_sum+=data[i]*data[i]
        return my_sum

    def end_and_save(self):
        self.is_recording=False
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        out_fname="sounds/sound_save_{}.wav".format(self.start_timestr)
        logger.debug("Saving {}".format(out_fname))
        wf = wave.open(out_fname, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.paudio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(b''.join(self.records))
        wf.close()
        self.records=[]

    def read_message(self,message):
        if "gyrus_config" in message and message["gyrus_config"]["target_gyrus"]=="SoundRecordGyrus":
            m=message["gyrus_config"]
            if m["command"]=="start":
                self.is_recording=True
                logger.debug("start recording")
                self.recording_start=time.time()

        if "microphone_data" in message:
            data=message["microphone_data"]
            energy=self.get_chunk_energy(data)
            self.avg_energy=self.avg_energy*self.avg_run+(1-self.avg_run)*energy
            if self.is_recording:
                if (time.time()>self.recording_start+self.min_sound_time) and energy<self.avg_energy:
                    self.end_and_save()
                else:
                    self.records.append(data)
