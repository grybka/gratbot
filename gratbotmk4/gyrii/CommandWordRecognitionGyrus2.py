from Gyrus import ThreadedGyrus
import os, pathlib
import logging
from hyperpyyaml import load_hyperpyyaml
import torch
import time
import numpy as np

from speechbrain.pretrained import EncoderClassifier
import torch.nn as nn
import logging

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

class WordRecognizer():
    def __init__(self,word_list):
        self.encoder=buildEncoderClassifier()
        self.word_list=word_list
        n_classes=len(self.word_list)
        input_size=512 #this is set by my encoder
        self.classifier=nn.Sequential(nn.Linear(input_size,2*n_classes),nn.Sigmoid(),nn.Linear(2*n_classes,n_classes))
        self.softmax=nn.Softmax(dim=1)

    def save_to_file(self,fname):
        torch.save({"classifier": self.classifier.state_dict()},fname)

    def load_from_file(self,fname):
        checkpoint=torch.load(fname)
        self.classifier.load_state_dict(checkpoint["classifier"])

    def classify_wave_tensor(self,t):
        rel_length=torch.tensor([1.0])
        xv = self.encoder.encode_batch(t,rel_length)
        #print("xv shape {}".format(xv.shape))
        logprobs=self.classifier(xv[0,:,:])
        #print("lpshape {}".format(logprobs.shape))
        return self.softmax(logprobs)[0,:].detach().numpy()

    def classify_wave(self,raw_bytes,sr):
        audio_int16 = np.frombuffer(raw_bytes, np.int16);
        unnorm=torch.from_numpy(audio_int16).float()
        unnorm.squeeze()
        data=self.encoder.audio_normalizer(unnorm,16000)
        return self.classify_wave_tensor(data)

    def guess_label(self,x):
        i=np.argmax(x)
        return self.word_list[i],x[i]



class CommandWordRecognitionGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        #TODO this should be in gyrus config
        target_words=["unknown","left","right","come","heel"]
        self.wordrecognizer=WordRecognizer(target_words)
        self.wordrecognizer.load_from_file("config/command_word_classifier.pt")

    def get_keys(self):
        return ["speech_detected"]

    def get_name(self):
        return "CommandWordRecognitionGyrus"

    def read_message(self,message):
        if "speech_detected" in message:
            data=b''.join(message["speech_detected"])
            word,score=self.wordrecognizer.guess_label(self.wordrecognizer.classify_wave(data,16000))
            #word,score=self.encoderclassifier.classify_wave(data)
            logger.debug("Recognized word -{}- with score {}".format(word,score))
