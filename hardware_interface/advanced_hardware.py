from hardware import GratbotSpimescape
from hardware import create_hardware_item
import pyaudio
import threading
import time
import logging
import base64
import copy

class GratbotAudio(GratbotSpimescape):
    #microphone, saves the last few seconds of audio when enabled
    def __init__(self,datastruct):
        self.save_audio_seconds=datastruct["save_audio_seconds"]
        self.num_past_buffers=datastruct["num_past_buffers"]
        self.format=pyaudio.paInt16
        self.sample_rate=44100
        self.chunk=4096
        self.dev_index=1 
        self.chans=1
        self.chunks_to_record=int((self.sample_rate/self.chunk)*self.save_audio_seconds)
        self.data_queue=threading.Queue(maxsize=self.chunks_to_record)

        #find device index with this
        self.paudio=pyaudio.PyAudio()
        #for ii in range(self.paudio.get_device_count()):
        #   print("device {} is {}".format(ii,self.paudio.get_device_info_by_index(ii).get('name')))
        #   print("device {} has a sample rate of {}".format(ii,self.paudio.get_device_info_by_index(ii)))

        self.is_recording=False
        self.started_recording=False
        self.thread_should_quit=False
        self.thread=threading.Thread(target=self._daemon_loop)
        self.thread.daemon=True
        self.thread.start()

    def start_recording(self):
        self.stream=self.paudio.open(format=self.format,rate=self.sample_rate,channels=self.chans,input_device_index=self.dev_index,input=True,frames_per_buffer=self.chunk)
        self.started_recording=True

    def stop_recording(self):
        self.stream.stop_stream()
        self.stream.close()

    def record_for_time(self):
        chunks_to_record=int((self.sample_rate/self.chunk)*self.save_audio_seconds)
        rec=self.stream.read(self.chunk)
        if self.data_queue.full():
            self.data_queue.get() #clear out old data if necessary
        self.data_queue.put(rec)
            
    def _daemon_loop(self):
        while not self.thread_should_quit:
            if self.is_recording:
                if not self.started_recording:
                    self.start_recording()
                self.record_for_time()
            else:
                time.sleep(0.1)

    def expose_endpoints(self):
        return [ ["sound_data"],["start_recording"] ]

    def set(self,endpoint,value):
        if endpoint=="start_recording":
            self.is_recording=True
        else:
            raise Exception("No endpoint {}".format(endpoint))

    def get(self,endpoint):
        if endpoint=="sound_data":
            toret=[]
            while not self.data_queue.empty():
                #so they are in reverse chronological order here
                toret.append(base64.b64encode(b''.join(self.data_queue.get())))
            return { "sound_data": toret.reverse() } #reversed to put them in right order here
        else:
            raise Exception("No endpoint {}".format(endpoint))
        
def create_advanced_hardware_item(datastruct):
    if "type" in datastruct:
        if datastruct["type"]=="GratbotAudio":
            return GratbotAudio(datastruct)
        else:
            return create_hardware_item(datastruct)

def create_advanced_hardware(datastruct):
    hardware_dat={}
    for x in datastruct.keys():
        logging.info("advanced hardware creating {}".format(x))
        hardware_dat[x]=create_advanced_hardware_item(datastruct[x])
    return hardware_dat

