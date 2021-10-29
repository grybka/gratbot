import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from tqdm import tqdm
import numpy as np

def gcc_phat(sigfft, refsigfft, fs=1, max_tau=None, interp=1):
    '''
    This function computes the offset between the signal sig and the reference signal refsig
    using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
    '''

    # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
    #n = sig.shape[0] + refsig.shape[0]
    n=2*sigfft.shape[0]

    # Generalized Cross Correlation Phase Transform
    #SIG = np.fft.rfft(sig, n=n)
    #REFSIG = np.fft.rfft(refsig, n=n)
    R = sigfft * np.conj(refsigfft)

    cc = np.fft.irfft(R / np.abs(R), n=(interp * n))

    max_shift = int(interp * n / 2)
    if max_tau:
        max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))

    # find max cross correlation index
    shift = np.argmax(np.abs(cc)) - max_shift

    tau = shift / float(interp * fs)
    return tau, cc

def all_taus(data):
    SOUND_SPEED = 340.0

    MIC_DISTANCE_4 = 0.081
    MAX_TDOA_4 = MIC_DISTANCE_4 / float(SOUND_SPEED)
    pairs=[ [0,1], [0,2], [0,3], [1,2], [1,3], [2,3] ]
    tau_mat=np.zeros([4,4])
    n=2*data.shape[0]
    signals_fft = np.fft.rfft(data, n=n,axis=0)
    for i, v in enumerate(pairs):
        #print(v)
        tau, _ = gcc_phat(signals_fft[:,v[0]],signals_fft[:,v[1]], 16000, max_tau=MAX_TDOA_4, interp=4)
        #print(tau)
        tau_mat[ v[0],v[1] ]=tau
        tau_mat[ v[1],v[0] ]=-tau
    return tau_mat

def angle_from_taus_better(tau_mat):
    # tau_mat[0,1]/tmaxsmall = sin(theta)
    # tau_mat[3,2]/tmaxsmall = sin(theta)
    # tau_mat[1,2]/tmaxsmall = cos(theta)
    # tau_mat[3,0]/tmaxsmall = cos(theta)
    # tau_mat[0,2]/tmaxlarge = sin(theta+45)
    # tau_mat[1,3]/tmaxlarge = cos(theta+45)
    g1=np.arctan2(tau_mat[0,1],tau_mat[1,2])
    g2=np.arctan2(tau_mat[3,2],tau_mat[0,3])
    g3=np.arctan2(tau_mat[0,2],tau_mat[1,3])-np.pi/4
    #print("guesses {} {} {}".format(np.degrees(g1),np.degrees(g2),np.degrees(g3)))
    #sowhewhere I have the sign wrong, I'm fixing it here
    sins=[np.sin(g1)+np.sin(g2)+np.sin(g3)]
    coss=[np.cos(g1)+np.cos(g2)+np.cos(g3)]


    sin_av=np.mean(sins)
    cos_av=np.mean(coss)
    return -np.arctan2(sin_av,cos_av)
    #return -np.arctan2( np.sin(g1)+np.sin(g2)+np.sin(g3),np.cos(g1)+np.cos(g2)+np.cos(g3))
    #return (g1+g2+g3)/3

def angle_from_audio(audio_data):
    return angle_from_taus_better(all_taus(audio_data))
