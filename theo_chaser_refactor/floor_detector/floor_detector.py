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
        self.conv.add_module("conv_1", torch.nn.Conv2d(3, 10, kernel_size=3,padding=[1,1]))
        self.conv.add_module("relu_1", torch.nn.ReLU())
        self.conv.add_module("norm_1",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_1", torch.nn.MaxPool2d(kernel_size=[1,2]))
        self.conv.add_module("conv_2", torch.nn.Conv2d(10, 10, kernel_size=3,padding=[1,1]))
        self.conv.add_module("dropout_2", torch.nn.Dropout())
        self.conv.add_module("relu_2", torch.nn.ReLU())
        self.conv.add_module("norm_2",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_2", torch.nn.MaxPool2d(kernel_size=[1,5]))
        self.conv.add_module("conv_3", torch.nn.Conv2d(10, 5, kernel_size=[3,5],padding=[1,2]))
        self.conv.add_module("relu_3", torch.nn.ReLU())
        self.conv.add_module("maxpool_3", torch.nn.MaxPool2d(kernel_size=[1,2]))
        self.conv.add_module("conv_4", torch.nn.Conv2d(5, 1, kernel_size=1))

        #self.linear = torch.nn.Sequential()
        #self.linear.add_module("linear_1",torch.nn.Linear(slice_height*32,slice_height*32))
        #self.linear.add_module("relu_1",torch.nn.ReLU())


    def forward(self, x):
        slice_height=240

        x = self.conv.forward(x)
        #print(x.shape)

        return x.view(-1,slice_height,32)
        #x=x.view(-1,slice_height*32)
        #x=self.linear.forward(x)
        #x=x.view(-1,slice_height,32)
        #return x

    def find_floor_pixels(thing,result_array,scale=4,start_bin=50,stop_bin=230):
        slice_height=240
        n_slices=32
        start_bin=0
        print(result_array.shape)
        mean_result=np.zeros(n_slices)
        stdev_result=np.zeros(n_slices)
        print(result_array)
        #scale slice
        for i in range(n_slices):
            psum=0
            xsum=0
            xxsum=0
            myscale=np.max(result_array[:,i])
            for j in range(start_bin,stop_bin):
                p=np.exp(scale*(result_array[j][i]-myscale))
                psum+=p
                xsum+=j*p
                xxsum+=j*j*p
            mymean=xsum/psum
            mystdev=np.sqrt(xxsum/psum-mymean*mymean)
            if mystdev<1:
                mystdev=1 #because pixels are quantized
            mean_result[i]=mymean
            stdev_result[i]=mystdev
        return mean_result,stdev_result


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

    def find_floor_pixels(self,result_array,scale=4,start_bin=50,stop_bin=230):
        slice_height=240
        n_slices=32
        start_bin=0
        print(result_array.shape)
        mean_result=np.zeros(n_slices)
        stdev_result=np.zeros(n_slices)
        print(result_array)
        #scale slice
        for i in range(n_slices):
            psum=0
            xsum=0
            xxsum=0
            myscale=np.max(result_array[:,i])
            for j in range(start_bin,stop_bin):
                p=np.exp(scale*(result_array[j][i]-myscale))
                psum+=p
                xsum+=j*p
                xxsum+=j*j*p
            mymean=xsum/psum
            mystdev=np.sqrt(xxsum/psum-mymean*mymean)
            if mystdev<1:
                mystdev=1 #because pixels are quantized
            mean_result[i]=mymean
            stdev_result[i]=mystdev
        return mean_result,stdev_result

    def get_floor_pixels(self,image):
        image_width=int(640)
        slice_width=int(40)
        image_height=480
        slice_height=240
        n_slices=32

        reduced_image=image[image_height-slice_height:image_height,:,:]
        X=torch.from_numpy(reduced_image).permute(2,0,1).float().unsqueeze(0)
        print("my shape {}".format(X.shape))
        Y=self.model.forward(X)
        print("my Y shape {}".format(Y.shape))
        return self.find_floor_pixels(Y[0])

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
