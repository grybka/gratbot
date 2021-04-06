
from Gyrus import ThreadedGyrus

from underpinnings.BayesianArray import BayesianArray

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
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import time



class FloorDetectionNet(torch.nn.Module):
    def __init__(self):
        self.slice_height=240
        slice_height=240
        super(FloorDetectionNet, self).__init__()

        self.conv = torch.nn.Sequential()
        self.conv.add_module("conv_1", torch.nn.Conv2d(3, 10, kernel_size=3,padding=[1,0]))
        self.conv.add_module("relu_1", torch.nn.ReLU())
        self.conv.add_module("norm_1",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_1", torch.nn.MaxPool2d(kernel_size=[1,2]))
        self.conv.add_module("conv_2", torch.nn.Conv2d(10, 10, kernel_size=5,padding=[2,0]))
        self.conv.add_module("dropout_2", torch.nn.Dropout())
        self.conv.add_module("relu_2", torch.nn.ReLU())
        self.conv.add_module("norm_2",torch.nn.BatchNorm2d(10))
        self.conv.add_module("maxpool_2", torch.nn.MaxPool2d(kernel_size=[1,5]))
        self.conv.add_module("conv_3", torch.nn.Conv2d(10, 1, kernel_size=[1,3],padding=[0,0]))
        #self.conv.add_module("relu_3", torch.nn.ReLU())
        #self.conv.add_module("maxpool_3", torch.nn.MaxPool2d(kernel_size=[1,2]))
        #self.conv.add_module("conv_4", torch.nn.Conv2d(5, 1, kernel_size=1))

        #self.linear = torch.nn.Sequential()
        #self.linear.add_module("linear_1",torch.nn.Linear(slice_height*32,slice_height*32))
        #self.linear.add_module("relu_1",torch.nn.ReLU())


    def forward(self, x):

        x = self.conv.forward(x)
        #print(x.shape)
        return x.view(-1,self.slice_height)

        #return x.view(-1,slice_height,32)
        #x=x.view(-1,slice_height*32)
        #x=self.linear.forward(x)
        #x=x.view(-1,slice_height,32)
        #return x

    def find_floor_pixels(self,result_array,scale=8.0,start_bin=2,end_bin=239):
        #print(result_array.shape)
        norm=np.max(result_array)
        #print("norm {}".format(norm))
        psum=0
        xsum=0
        xxsum=0
        for i in range(len(result_array)):
            #print("val {}".format(result_array[i]))
            #print("scale {}".format(scale))
            #print("norm {}".format(norm))
            #print("exponent {}".format(np.exp(scale*result_array[i]-norm)))
            p=np.exp(scale*(result_array[i]-norm))
            psum+=p
            xsum+=i*p
            xxsum+=i*i*p
        #print("xsum {}".format(xsum))
        mymean=xsum/psum
        mystdev=np.sqrt(xxsum/psum-mymean*mymean)
        if mystdev<1:
            mystdev=1 #because pixels are quantized
        return mymean,mystdev

def get_distance_from_pixels(centers,means,stdevs,image_width,image_height,x_unc=0,camera_height=0,horizon_offset=0,camera_tilt=0,camera_focal_length_pixels=0):
    #camera_focal_length_pixels=727
    means_with_unc=unumpy.uarray(means,stdevs)
    centers_with_unc=unumpy.uarray(centers,np.ones(len(centers))*x_unc)
    x_angles=unumpy.arctan((centers_with_unc-image_width/2)/camera_focal_length_pixels)
    horizon_offset_array=horizon_offset+unumpy.nominal_values(x_angles)*camera_tilt
    y_angles=unumpy.arctan(unumpy.cos(x_angles)*(means_with_unc-image_height/2+horizon_offset_array)/camera_focal_length_pixels)
    dists=camera_height/unumpy.tan(y_angles)
    return x_angles,dists

class FloorDetector:
    def __init__(self):
        self.model=FloorDetectionNet()
        self.slice_height=240
        #self.camera_hfov=45.0*(2*np.pi)/360 #corrected by hand
        #self.camera_wfov=62.2*(2*np.pi)/360 #from spec sheet
        self.camera_focal_length_pixels=531 #from wfov
        #self.camera_focal_length_pixels=727 #from wfov
        self.camera_height=0.08 #meters, measured with tape measure
        self.camera_tilt=0
        #self.horizon_offset=-77.0 #calibrated in a notebook.  TODO add this in-situ calibration for when camera shakes
        self.horizon_offset=-65.0 #calibrated in a notebook.  TODO add this in-situ calibration for when camera shakes

    def get_floor_loc_pixels(self,image):
        y_pick=240
        n_slices=31
        image_width=640
        slice_width=40
        xstack=[]
        for i in range(n_slices):
            x_start=int(i*image_width/(n_slices+1))
            x_stop=x_start+slice_width
            sub_image=image[y_pick:y_pick+self.slice_height,x_start:x_stop,:]
            X=torch.from_numpy(sub_image).permute(2,0,1).float()
            xstack.append(X)
            #print(X.shape)
        xstack=torch.stack(xstack)
        fX=self.model.forward(xstack).cpu().detach().numpy()
        #print(fX.shape)

        means=[]
        stdevs=[]
        xs=[]
        for i in range(n_slices):
            x_start=int(i*image_width/n_slices)
            x_stop=x_start+slice_width
            m,s=self.model.find_floor_pixels(fX[i])
            means.append(m+y_pick)
            stdevs.append(s)
            xs.append(0.5*(x_start+x_stop))
        return np.array(xs),np.array(means),np.array(stdevs)

    def get_floor_distances(self,image,max_pixel_stdev=100):
        image_width=image.shape[1]
        image_height=image.shape[0]
        centers,means,stdevs=self.get_floor_loc_pixels(image)
        center_unc=centers[1]-centers[0]
        centers=centers[stdevs<max_pixel_stdev]
        means=means[stdevs<max_pixel_stdev]
        stdevs=stdevs[stdevs<max_pixel_stdev]
        self.last_pixel_means=means
        self.last_pixel_centers=centers #For use in debug plotting
        self.last_pixel_stdevs=stdevs

        return  get_distance_from_pixels(centers,means,stdevs,image_width,image_height,x_unc=center_unc,camera_height=self.camera_height,horizon_offset=self.horizon_offset,camera_tilt=self.camera_tilt,camera_focal_length_pixels=self.camera_focal_length_pixels)

class FloorDetectorGyrus(ThreadedGyrus):
    def __init__(self,broker,debugshow=False,checkpoint_fname="floor_detector_sliced.pt",max_ptptvariance=0.4,min_distance=0.2,max_unc_fraction=0.1):
        self.min_distance=min_distance #do not keep distances closer than this
        self.max_ptptvariance=max_ptptvariance # do not keep points that vary by this much between adjacent angles
        self.max_unc_fraction=max_unc_fraction # do not keep points with larger than this fractional uncertainty
        self.max_pixel_stdev=5
        self.min_pts_for_success=5
        self.floordetector_checkpoint_fname=checkpoint_fname
        self.floordetector=FloorDetector()
        self.camera_height=0.0987  #in cm, calibrated
        self.horizon_offset=-26.1 #calibrated in a notebook.  TODO add this in-situ calibration for when camera shakes
        self.camera_tilt=8.64 # calibrated
        self.floordetector.camera_height=self.camera_height
        self.floordetector.horizon_offset=self.horizon_offset
        self.floordetector.camera_tilt=self.camera_tilt
        checkpoint = torch.load(self.floordetector_checkpoint_fname)
        self.floordetector.model.load_state_dict(checkpoint['model_state_dict'])
        self.floordetector.model.eval()
        self.debugshow=debugshow
        self.last_pose=None

        self.clear_frames_before=0
        super().__init__(broker)

    def get_keys(self):
        return [ "camera_frame","latest_pose" ]

    def get_name(self):
        return "FloorWallGyrus"

    def read_message(self,message):
        if "latest_pose" in message:
            self.last_pose=BayesianArray.from_object(message["latest_pose"])
        if "camera_frame" in message:
            if self.last_pose==None:
                return #in case I start too early
            if message["timestamp"]<self.clear_frames_before:
                #print("skipping floorwall")
                return #If this is true, I've built up a backlog of frames.  Throw them away and use a recent one
            #print("processing floorwall")
            image=message["camera_frame"]
            x_angles,dists=self.floordetector.get_floor_distances(image,max_pixel_stdev=self.max_pixel_stdev)
            x_angles,dists=self.prune_dists_and_angles(x_angles,dists)
            if len(x_angles)<self.min_pts_for_success:
                if self.debugshow:
                    print("only {} points found, no floowall claim".format(len(x_angles)))
                return []
            myinfo={"x_angles": unumpy.nominal_values(x_angles).tolist(),"x_angles_unc": unumpy.std_devs(x_angles).tolist(),"dists":unumpy.nominal_values(dists).tolist(),"dists_unc":unumpy.std_devs(dists).tolist(),"last_pose": self.last_pose.to_object()}
            mymessage={"timestamp": message["timestamp"],"floor_detector_measurement":myinfo}
            self.broker.publish(mymessage,"floor_detector_measurement")
            if self.debugshow==True:
                self.show_debug_plots(x_angles,dists,image)
            self.clear_frames_before=time.time()+0.5 #maybe I should record elapsed time

    def prune_dists_and_angles(self,x_angles,dists):
        #remove any pair of points that vary by too much or are too far away
        if self.debugshow:
            print("input to point pruning {} pts".format(len(x_angles)))
            print("x_angles {}".format(x_angles))
            print("dists {}".format(dists))
        keep_array=dists>self.min_distance
        max_vary=self.max_ptptvariance

        for k in range(len(x_angles)-1):
            if abs(dists[k]-dists[k+1])>max_vary:
                keep_array[k]=False
                keep_array[k+1]=False
        valid_angles=x_angles[ keep_array]
        valid_dists=dists[ keep_array]
        if self.debugshow:
            print("after max vary {} pts".format(len(valid_angles)))
        x_angles=valid_angles
        dists=valid_dists
        #remove any ponits with too big of errors
        uncs=abs(unumpy.std_devs(dists)/unumpy.nominal_values(dists))
        max_dist_unc_fraction=self.max_unc_fraction
        valid_angles=x_angles[ uncs<max_dist_unc_fraction]
        valid_dists=dists[ uncs<max_dist_unc_fraction]
        if self.debugshow:
            print("after max uncfraction {} pts".format(len(valid_angles)))
        x_angles=valid_angles
        dists=valid_dists
        return x_angles,dists

    def show_debug_plots(self,x_angles,dists,image):
        fig,ax = plt.subplots(1)
        #plt.imshow(image[240:480,:,:])
        plt.imshow(image)
        slice_width=40
        centers=self.floordetector.last_pixel_centers
        means=self.floordetector.last_pixel_means
        stdevs=self.floordetector.last_pixel_stdevs
        for i in range(len(self.floordetector.last_pixel_centers)):
            xs=[centers[i]-slice_width/2,centers[i]+slice_width/2]
            ys=[means[i],means[i]]
            plt.plot(xs,ys)
            ax.add_patch(patches.Rectangle((centers[i]-20,means[i]-stdevs[i]*0.5),40,stdevs[i],linewidth=1,edgecolor='r',facecolor='none'))
        plt.show()

        plt.figure(figsize=(6,6))
        plt.axis("equal")
        xs=unumpy.nominal_values(dists)*np.sin(unumpy.nominal_values(x_angles))
        ys=unumpy.nominal_values(dists)*np.cos(unumpy.nominal_values(x_angles))
        plt.plot([ys, np.zeros(np.size(ys))], [xs, np.zeros(np.size(ys))], "ro-") # lines from 0,0 to the

        #plt.plot(xs,ys,'*')

        #plt.polar(unumpy.nominal_values(x_angles),unumpy.nominal_values(dists),'*')
        plt.show()
