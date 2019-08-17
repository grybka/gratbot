from hardware import GratbotSpimescape
from hardware import create_hardware_item
import pyaudio
import threading
import time

class GratbotAudio(GratbotSpimescape):
    #microphone, saves the last few seconds of audio when enabled
    def __init__(self,datastruct):
        self.save_audio_seconds=datastruct["save_audio_seconds"]
        self.num_past_buffers=datastruct["num_past_buffers"]
        self.format=pyaudio.paInt16
        self.sample_rate=44100
        self.chunk=4096
        self.dev_index=1 
        #find device index with this
        #for ii in range(p.get_device_count()):
        #   print("device {} is {}".format(ii,p.get_device_info_by_index(ii).get('name')))
        self.paudio=pyaudio.PyAudio()
        self.past_buffers=[]
        self.past_buffers_lock=threading.Lock()
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
        recording=[]
        for i in range(chunks_to_record):
            recording.extend(self.stream.read(self.chunk))
        #TODO any processing on this recording?
        self.past_buffers_lock.acquire()
        self.past_buffers.insert(0,recording)
        if len(self.past_buffers)>self.num_past_buffers:
            self.past_buffers.pop()
        self.past_buffers_lock.release()
            
    def _daemon_loop():
        while not self.thread_should_quit:
            if self.is_recording:
                if not self.started_recording:
                    self.start_recording():
                record_for_time()
            else:
                time.sleep(0.1)
        
def create_advanced_hardware_item(datastruct):
    if "type" in datastruct:
        logging.info("advanced hardware creating {}".format(x))
        if datastruct["type"]=="GratbotAudio":
            return GratbotAudio()
        else:
            return create_hardware_item(datastruct):

def create_advanced_hardware(datastruct):
    hardware_dat={}
    for x in datastruct.keys():
        hardware_dat[x]=create_advanced_hardware_item(datastruct[x])
    return hardware_dat

