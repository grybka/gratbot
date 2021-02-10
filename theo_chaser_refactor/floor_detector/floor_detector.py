import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

import torch
from torch.autograd import Variable
from torch import optim
from uncertainties import ufloat
from uncertainties.umath import *
from uncertainties import unumpy


class FloorDetectionNet(torch.nn.Module):
    def __init__(self):
        slice_height=240
        super(FloorDetectionNet, self).__init__()

        self.conv = torch.nn.Sequential()
        self.conv.add_module("conv_1", torch.nn.Conv2d(3, 10, kernel_size=4))
        self.conv.add_module("relu_1", torch.nn.ReLU())
        self.conv.add_module("norm_1",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_1", torch.nn.MaxPool2d(kernel_size=[1,2]))
        self.conv.add_module("conv_2", torch.nn.Conv2d(10, 10, kernel_size=4,padding=[3,0]))
        self.conv.add_module("dropout_2", torch.nn.Dropout())
        self.conv.add_module("maxpool_2", torch.nn.MaxPool2d(kernel_size=[1,4]))
        self.conv.add_module("relu_2", torch.nn.ReLU())
        self.conv.add_module("norm_2",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_3", torch.nn.MaxPool2d(kernel_size=[1,3]))
        self.conv.add_module("conv_3", torch.nn.Conv2d(10, 1, kernel_size=1))


        #self.fc = torch.nn.Sequential()
        #self.fc.add_module("fc1", torch.nn.Linear(10*(slice_height-6)*1, slice_height))
        #self.fc.add_module("relu_3", torch.nn.ReLU())
        ##self.fc.add_module("dropout_3", torch.nn.Dropout())
        #self.fc.add_module("fc2", torch.nn.Linear(slice_height, slice_height))

    def forward(self, x):
        slice_height=240
        #print(x.shape)
        x = self.conv.forward(x)
        #print(x.shape)
        #x = x.view(-1, 10*54*1)
        #x = x.view(-1, 10*(slice_height-6)*1)
        #return self.fc.forward(x)
        #return x.view(-1,54)
        return x.view(-1,slice_height)

    def find_floor_pixel(self,result_array,scale=3.0,start_bin=2):
        psum=0
        xsum=0
        xxsum=0
        for i in range(start_bin,len(result_array)):
            p=np.exp(scale*result_array[i])
            psum+=p
            xsum+=i*p
            xxsum+=i*i*p
        mymean=xsum/psum
        mystdev=np.sqrt(xxsum/psum-mymean*mymean)
        if mystdev<1:
            mystdev=1 #because pixels are quantized
        return mymean,mystdev

class FloorDetector:
    def __init__(self):
        self.model=FloorDetectionNet()
        #self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        #self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        #self.camera_focal_length_pixels=531 #from wfov
        self.camera_focal_length_pixels=727 #from wfov
        self.camera_height=0.08 #meters, measured with tape measure
        #self.horizon_offset=-77.0 #calibrated in a notebook.  TODO add this in-situ calibration for when camera shakes
        self.horizon_offset=-65.0 #calibrated in a notebook.  TODO add this in-situ calibration for when camera shakes

    def get_floor_pixels(self,image):
        image_width=int(640)
        slice_width=int(40)
        image_height=480
        slice_height=240
        #height_slices=[0,30,60,90,120]
        #break into segments
        torch_segments=[]
        for i in range(int(image_width/slice_width)):
            torch_segments.append(torch.from_numpy(image[image_height-slice_height:image_height,i*slice_width:(i+1)*slice_width,:]).permute(2,0,1).float())
            #for j in range(len(height_slices)):
            #    torch_segments.append(torch.from_numpy(image[image_height-slice_height-height_slices[j]:image_height-height_slices[j],i*slice_width:(i+1)*slice_width,:]).permute(2,0,1).float())
        torch_array=torch.stack(torch_segments)
        self.model.eval()
        output=self.model.forward(torch_array)
        output=output.detach().numpy()
        centers=[]
        means=[]
        stdevs=[]
        for i in range(int(image_width/slice_width)):
            centers.append((i+0.5)*slice_width)
            m,s=self.model.find_floor_pixel(output[i])
            m+=slice_height
            means.append(m)
            stdevs.append(s)
        means=np.array(means)
        stdevs=np.array(stdevs)
        centers=np.array(centers)
        return centers,means,stdevs

    def get_distance_from_pixels(self,centers,means,stdevs,image_width,image_height,x_unc=0):
        means_with_unc=unumpy.uarray(means,stdevs)
        centers_with_unc=unumpy.uarray(centers,np.ones(len(centers))*x_unc)
        x_angles=unumpy.arctan((centers_with_unc-image_width/2)/self.camera_focal_length_pixels)
        #x_unc=(x_angles[1]-x_angles[0])
        #print("x angles {}".format(x_angles*360/(2*3.14)))
        y_angles=unumpy.arctan(unumpy.cos(x_angles)*(means_with_unc-image_width/2+self.horizon_offset)/self.camera_focal_length_pixels)

        #print("y angles {}".format(y_angles*360/(2*3.14)))
        dists=self.camera_height/unumpy.tan(y_angles)
        return x_angles,dists

    def get_floor_distances(self,image,max_pixel_stdev=100):
        image_width=image.shape[1]
        image_height=image.shape[0]
        centers,means,stdevs=self.get_floor_pixels(image)
        center_unc=centers[1]-centers[0]
        centers=centers[stdevs<max_pixel_stdev]
        means=means[stdevs<max_pixel_stdev]
        stdevs=stdevs[stdevs<max_pixel_stdev]
        return self.get_distance_from_pixels(centers,means,stdevs,image_width,image_height,x_unc=center_unc)
