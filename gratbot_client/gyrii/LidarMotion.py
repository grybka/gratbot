#Lidar movement tracker wit ICP

from Gyrus import ThreadedGyrus
import cv2 as cv
import time
import numpy as np
from underpinnings.OccupancyMap2 import ExactLidarMeasurement
from underpinnings.ICP import Align2D
from gyrii.underpinnings.GratbotLogger import gprint

class LidarICPTracker(ThreadedGyrus):
    def __init__(self,broker,display=None):
        self.clear_frames_before=0
        self.n_dropped_frames=0
        super().__init__(broker)

        self.last_scan_points=None
        self.last_time=0

        #profiling
        self.profiling=True
        #self.profiling=False
        self.n_dropped_frames=0
        self.n_sample=20
        self.time_sum=0
        self.sample_sum=0

    def get_keys(self):
        return [ "lidar/lidar_scan" ]

    def get_name(self):
        return "VisualOdometerConv"

    def update(self,lidar_scan,timestamp):
        n_points=90
        if self.last_scan_points is None:
            self.last_scan_points=lidar_scan.to_xypoints_extraone_with_downsample(n_points)
            self.last_time=timestamp
            return
        source_points=self.last_scan_points
        target_points=lidar_scan.to_xypoints_extraone_with_downsample(n_points)
        align=Align2D(source_points,target_points,np.identity(3),max_iter=12,max_delta=1e-3)
        transform=align.transform
        angle=np.arctan2(transform[0][1],transform[0][0])
        #gprint("Trasnsform is {}".format(transform))
        #if abs(angle)>0.05 or align.num_iter>3:
        if abs(angle)>0.05:
            gprint("Transform angle is {}".format(np.arctan2(transform[0][1],transform[0][0])))
            gprint("Error, delta error, num iter  {} , {} , {}".format(align.mean_sq_error,align.delta_err,align.num_iter))
        self.last_scan_points=target_points

    def read_message(self,message):
        if "lidar/lidar_scan" in message:
            if message["timestamp"]<self.clear_frames_before:
                self.n_dropped_frames+=1
                return
            start_time=time.time()
            scan_data=message["lidar/lidar_scan"]
            angles=[ np.radians(x[1]) for x in scan_data ]
            dists=[ x[2]/1000 for x in scan_data ]
            m=ExactLidarMeasurement(dists,angles)
            self.update(m,message["timestamp"])
            self.clear_frames_before=time.time()
            if self.profiling:
                self.time_sum+=time.time()-start_time
                self.sample_sum+=1
                if self.sample_sum>=self.n_sample:
                    gprint("average lidar icp time {} ms".format(1000*self.time_sum/self.sample_sum))
                    gprint("number of dropped lidar frames per used {}".format(self.n_dropped_frames/self.sample_sum))
                    self.n_dropped_frames=0
                    self.sample_sum=0
                    self.time_sum=0
