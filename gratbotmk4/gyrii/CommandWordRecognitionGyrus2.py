from Gyrus import ThreadedGyrus
import os, pathlib
import logging
from hyperpyyaml import load_hyperpyyaml
import torch
import time
import numpy as np
import wave

from speechbrain.pretrained import EncoderClassifier
import torch.nn as nn
import logging

from underpinnings.AudioDOA import angle_from_audio

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


#I'm going to use just the encoder part of speechbrain and make my own classifier

def buildEncoderClassifier():
    hparams_local_path=pathlib.Path(os.getcwd()+'\pretrained_models\google_speech_command_xvector\hyperparams.yaml')
    savedir=pathlib.Path(os.getcwd()+'\pretrained_models\google_speech_command_xvector')
    kwargs={}
    overrides={}
    with open(hparams_local_path) as fin:
        hparams = load_hyperpyyaml(fin, overrides)

    # Pretraining:
    pretrainer = hparams["pretrainer"]
    pretrainer.set_collect_in(savedir)
    pretrainer.load_collected(device="cpu")

    # Now return the system
    return EncoderClassifier(hparams["modules"], hparams, **kwargs)

import torch.nn as nn

class WordClassifier(nn.Module):
    def __init__(self,input_size,n_words,n_speakers):
        super(WordClassifier, self).__init__()
        n_middle=2*max(n_words,n_speakers)
        self.layer1=nn.Sequential(nn.Linear(input_size,n_middle),nn.Sigmoid())
        self.word_layer=nn.Linear(n_middle,n_words)
        self.speaker_layer=nn.Linear(n_middle,n_speakers)

    def forward(self,x):
        mid=self.layer1(x)
        return self.word_layer(mid),self.speaker_layer(mid)



class WordRecognizer():
    def __init__(self,word_list=None,speaker_list=None):
        self.encoder=buildEncoderClassifier()
        self.word_list=word_list
        self.speaker_list=speaker_list
        self.input_size=512 #this is set by my encoder
        #self.classifier=nn.Sequential(nn.Linear(input_size,2*n_classes),nn.Sigmoid(),nn.Linear(2*n_classes,n_classes))
        if word_list is not None:
            self.classifier=WordClassifier(self.input_size,len(self.word_list),len(self.speaker_list))
        self.softmax=nn.Softmax(dim=1)

    def save_to_file(self,fname):
        torch.save({"classifier": self.classifier.state_dict(),
                    "word_list": self.word_list,
                    "speaker_list": self.speaker_list},fname)

    def load_from_file(self,fname):
        checkpoint=torch.load(fname)
        self.word_list=checkpoint["word_list"]
        self.speaker_list=checkpoint["speaker_list"]
        self.classifier=WordClassifier(self.input_size,len(self.word_list),len(self.speaker_list))
        self.classifier.load_state_dict(checkpoint["classifier"])
        self.classifier.eval()

    def classify_wave_tensor(self,t):

        t=self.encoder.audio_normalizer(t,16000)
        rel_length=torch.tensor([1.0])
        xv = self.encoder.encode_batch(t,rel_length)
        #print("xv shape {}".format(xv.shape))
        logprobs_words,logprobs_speakers=self.classifier(xv[0,:,:])
        #print("lpshape {}".format(logprobs.shape))
        return self.softmax(logprobs_words)[0,:].detach().numpy(),self.softmax(logprobs_speakers)[0,:].detach().numpy()

    def bytes_to_tensor(self,raw_bytes,sr):
        audio_int16 = np.frombuffer(raw_bytes, np.int16);
        audio_int16=np.reshape(audio_int16,(-1,4)) #four channels
        #audio_int16 = np.frombuffer(raw_bytes, np.int16);
        #audio_int16=audio_int16[:,0]
        unnorm=torch.from_numpy(audio_int16).float()
        #unnorm.squeeze()
        #logger.debug("unnomrm shape {}".format(unnorm.shape))
        return unnorm

    def classify_wave(self,raw_bytes,sr):
        audio_int16 = np.frombuffer(raw_bytes, np.int16);
        audio_int16=np.reshape(audio_int16,(-1,4)) #four channels
        #audio_int16 = np.frombuffer(raw_bytes, np.int16);
        audio_int16=audio_int16[:,0]
        unnorm=torch.from_numpy(audio_int16).float()
        unnorm.squeeze()
        #data=self.encoder.audio_normalizer(unnorm,16000)
        return self.classify_wave_tensor(data)

    def guess_label(self,x):
        i=np.argmax(x)
        return self.word_list[i],float(x[i])

    def guess_speaker(self,x):
        i=np.argmax(x)
        return self.speaker_list[i],float(x[i])

    def sorted_label_guess(self,x):
        the_is=np.flip(np.argsort(x))
        words=np.array(self.word_list)[the_is]
        xs=x[the_is]
        return words,xs



class CommandWordRecognitionGyrus(ThreadedGyrus):
    def __init__(self,broker,save_to_file=True):
        super().__init__(broker)
        #TODO this should be in gyrus config
        #target_words=["unknown","left","right","come","heel","stop","no","go","robot"]

        self.wordrecognizer=WordRecognizer()
        self.wordrecognizer.load_from_file("config/command_word_classifier.pt")
        self.save_words=save_to_file


    def get_keys(self):
        return ["speech_detected"]

    def get_name(self):
        return "CommandWordRecognitionGyrus"

    def save_word_sample(self,data,word,speaker):
        self.start_timestr = time.strftime("%Y%m%d-%H%M%S")
        out_fname="sounds/{}_{}_{}.wav".format(speaker,word,self.start_timestr)
        logger.debug("Saving {}".format(out_fname))
        wf = wave.open(out_fname, 'wb')
        wf.setnchannels(4)
        #wf.setsampwidth(self.paudio.get_sample_size(pyaudio.paInt16))
        wf.setsampwidth(2)
        wf.setframerate(16000)
        wf.writeframes(b''.join(data))
        wf.close()

    def read_message(self,message):
        if "speech_detected" in message:
            data=b''.join(message["speech_detected"])
            data_as_tensor=self.wordrecognizer.bytes_to_tensor(data,16000)
            logger.debug("data as tensor shape {}".format(data_as_tensor.shape))
            angle_prediction=angle_from_audio(data_as_tensor)
            logger.debug("angle: {}".format(np.degrees(angle_prediction)))

            angpix=int(angle_prediction*12/(2*np.pi)-3)%12
            my_rgb=[]
            for i in range(12):
                if i==angpix:
                    my_rgb.append([75,75,75])
                else:
                    my_rgb.append([0,0,0])
            self.broker.publish({"led_command":{"rgb_brightness": my_rgb}},"led_command")

            #output_word,output_speaker=self.wordrecognizer.classify_wave(data,16000)
            output_word,output_speaker=self.wordrecognizer.classify_wave_tensor(data_as_tensor[:,0])
            topwords,topwordscores=self.wordrecognizer.sorted_label_guess(output_word)
            for i in range(3):
                logger.debug("{}: {} ({})".format(i,topwords[i],topwordscores[i]))
            word,score=self.wordrecognizer.guess_label(output_word)
            speaker,score=self.wordrecognizer.guess_speaker(output_speaker)
            #word,score=self.encoderclassifier.classify_wave(data)
            logger.debug("Recognized word -{}- with score {}".format(word,score))
            logger.debug("Recognized speaker -{}- with score {}".format(speaker,score))
            self.broker.publish({"timestamp": time.time(),"command_received": {"command": word, "confidence": score}},["command_received"])
            if self.save_words:
                self.save_word_sample(message["speech_detected"],word,speaker)
