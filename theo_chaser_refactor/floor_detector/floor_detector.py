import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

import torch
from torch.autograd import Variable
from torch import optim

class FloorDetectionNet(torch.nn.Module):
    def __init__(self):
        super(FloorDetectionNet, self).__init__()

        self.conv = torch.nn.Sequential()
        self.conv.add_module("conv_1", torch.nn.Conv2d(3, 10, kernel_size=4))
        self.conv.add_module("maxpool_1", torch.nn.MaxPool2d(kernel_size=[1,2]))
        self.conv.add_module("relu_1", torch.nn.ReLU())
        self.conv.add_module("conv_2", torch.nn.Conv2d(10, 10, kernel_size=4))
        self.conv.add_module("dropout_2", torch.nn.Dropout())
        self.conv.add_module("maxpool_2", torch.nn.MaxPool2d(kernel_size=[1,4]))
        self.conv.add_module("relu_2", torch.nn.ReLU())
        self.conv.add_module("maxpool_3", torch.nn.MaxPool2d(kernel_size=[1,3]))


        self.fc = torch.nn.Sequential()
        self.fc.add_module("fc1", torch.nn.Linear(10*54*1, 60))
        self.fc.add_module("relu_3", torch.nn.ReLU())
        #self.fc.add_module("dropout_3", torch.nn.Dropout())
        self.fc.add_module("fc2", torch.nn.Linear(60, 60))

    def forward(self, x):
        #print(x.shape)
        x = self.conv.forward(x)
        #print(x.shape)
        x = x.view(-1, 10*54*1)
        return self.fc.forward(x)
