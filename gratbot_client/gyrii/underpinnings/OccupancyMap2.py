
#an implimentation of an occupancy map
#more in keeping with the standard log-odds practice than the previous one
import numpy as np
from underpinnings.BayesianArray import BayesianArray
from underpinnings.GridGeometry import triangle_inside
from underpinnings.GridGeometry import bresenham
from uncertainties import ufloat
from uncertainties.umath import *
from uncertainties import unumpy
import scipy

import threading
#for plotting
#from matplotlib import pyplot as plt
import cv2 as cv


class ExactLidarMeasurement:
    def __init__(self,dists,angles):
        self.dists=dists
        self.angles=angles

    def to_xypoints(self,exactpose): #exactpose [x,y,theta]
        xs=exactpose[0]*np.ones(len(self.angles))+self.dists*np.sin(self.angles+exactpose[2])
        ys=exactpose[1]*np.ones(len(self.angles))+self.dists*np.cos(self.angles+exactpose[2])
        return np.column_stack((xs,ys))

class LidarMeasurement:
    def __init__(self,dists,dist_uncs,angles,angle_uncs):
        self.dists=dists
        self.dist_uncs=dist_uncs
        self.angles=angles
        self.angle_uncs=angle_uncs

    def sample_to_exact(self):
        return ExactLidarMeasurement(np.random.normal(self.dists,self.dist_uncs),np.random.normal(self.angles,self.angle_uncs))


    def to_xypoints(self,pose): #pose is a BayesianArray
        [posex,posey,posetheta]=pose.get_as_ufloat()
        dists=unumpy.uarray(self.dists,self.dist_uncs)
        angles=unumpy.uarray(self.angles,self.angle_uncs)
        thetas=posetheta*np.ones(len(self.angles))+angles
        centerxs=posex*np.ones(len(self.angles))+dists*unumpy.sin(thetas)
        centerys=posey*np.ones(len(self.angles))+dists*unumpy.cos(thetas)
        ret=[]
        for cx,cy in zip(centerxs,centerys):
            ret.append(BayesianArray.from_ufloats([cx,cy]))
        return ret

    def to_xypoints_with_fixed_offset(self,xoffset,yoffset,angleoffset): #pose is a BayesianArray
        #[posex,posey,posetheta]=pose.get_as_ufloat()
        dists=unumpy.uarray(self.dists,self.dist_uncs)
        angles=unumpy.uarray(self.angles,self.angle_uncs)
        thetas=angleoffset*np.ones(len(self.angles))+angles
        centerxs=xoffset*np.ones(len(self.angles))+dists*unumpy.sin(thetas)
        centerys=yoffset*np.ones(len(self.angles))+dists*unumpy.cos(thetas)
        ret=[]
        for cx,cy in zip(centerxs,centerys):
            ret.append(BayesianArray.from_ufloats([cx,cy]))
        return ret

    def __len__(self):
        return len(self.dists)


class OccupancyMap2:

    def __init__(self,resolution,npoints_x,npoints_y):
        self.resolution=resolution #grid size in meters
        self.npoints_x=npoints_x
        self.npoints_y=npoints_y
        self.gridmap_occupied_lock=threading.Lock()
        self.gridmap_logodds=np.zeros([self.npoints_x,self.npoints_y])
        self.occupied_logodds=0.8
        self.free_logodds=-0.2
        self.logodds_clip=5


    def coords_to_cells(self,pts):
        #TODO this could probably be faster
        ret=[]
        for pt in pts:
            ret.append(self.coord_to_cell(pt))
        return ret

    def coord_to_cell(self,pt):
        xcell=min(max(int( (pt[0]+self.resolution*self.npoints_x/2)/self.resolution),0),self.npoints_x-1)
        ycell=min(max(int( (pt[1]+self.resolution*self.npoints_y/2)/self.resolution),0),self.npoints_y-1)
        #return np.array([xcell,ycell])
        return (xcell,ycell)

    def cell_to_coord(self,pt):
        xpt=(pt[0]-self.npoints_x/2)*self.resolution
        ypt=(pt[1]-self.npoints_y/2)*self.resolution
        return np.array([xpt,ypt])

    def apply_lidar_measurement(self,pose,mlidar,n_draws=10):
        weight=1/n_draws
        for i in range(n_draws):
            #exactpose=np.array(pose.vals)
            exactpose=pose.random_sample()
            #exactlidar=ExactLidarMeasurement(mlidar.dists,mlidar.angles)
            exactlidar=mlidar.sample_to_exact()
            self.apply_exact_lidar_measurement(exactpose,exactlidar,weight=weight)

    def apply_exact_lidar_measurement(self,exactpose,exactlidar,weight=1):
        end_pts=exactlidar.to_xypoints(exactpose)
        start_cell=self.coord_to_cell(exactpose[0:2])
        end_cells=self.coords_to_cells(end_pts)
        all_cells=set()
        for e in end_cells:
            all_cells.update(bresenham(start_cell,e,returnset=True))
        end_cells_set=set(end_cells)
        free_cells=all_cells.difference(end_cells_set)
        with self.gridmap_occupied_lock:
            for cell in free_cells:
                self.gridmap_logodds[cell[0],cell[1]]=np.clip(self.gridmap_logodds[cell[0],cell[1]]+weight*self.free_logodds,-self.logodds_clip,self.logodds_clip)
            for cell in set(end_cells):
                self.gridmap_logodds[cell[0],cell[1]]=np.clip(self.gridmap_logodds[cell[0],cell[1]]+weight*self.occupied_logodds,-self.logodds_clip,self.logodds_clip)

    def occupancy_to_image(self,pose):
        with self.gridmap_occupied_lock:
            ret=np.exp(self.gridmap_logodds.T[::-1,:])
        ret=255*(np.ones(ret.shape)-np.ones(ret.shape)/(np.ones(ret.shape)+ret))
        map_image=cv.applyColorMap(ret.astype(np.uint8),cv.COLORMAP_HOT)
        center=self.coord_to_cell(pose.vals)
        startpt=(center[0],self.npoints_y-center[1]) #remember I have to flip the y axis
        arrowpt=self.coord_to_cell(pose.vals[0:2]+np.array([0.2*sin(pose.vals[2]),0.2*cos(pose.vals[2])]))
        cv.arrowedLine(map_image,startpt,(arrowpt[0],self.npoints_y-arrowpt[1]),(255,255,255),1, cv.LINE_AA, 0, 0.3)
        return cv.resize(map_image,(600,600))
