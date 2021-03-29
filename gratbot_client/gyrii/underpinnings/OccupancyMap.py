#an implimentation of an occupancy map
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

class LidarMeasurement:
    def __init__(self,dists,dist_uncs,angles,angle_uncs):
        self.dists=dists
        self.dist_uncs=dist_uncs
        self.angles=angles
        self.angle_uncs=angle_uncs

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

class OccupancyMap:
    def __init__(self,resolution,npoints_x,npoints_y):
        self.resolution=resolution #grid size in meters
        self.npoints_x=npoints_x
        self.npoints_y=npoints_y
        #self.gridmap_free=0.9*np.ones([self.npoints_x,self.npoints_y])
        self.gridmap_occupied=0.1*np.ones([self.npoints_x,self.npoints_y])
        self.gridmap_explored=np.zeros([self.npoints_x,self.npoints_y])
        self.gridmap_explored_max=5
        self.max_counts=5
        self.gridmap_occupied_lock=threading.Lock()
        #self.gridmap_prob=0.1*np.ones([self.npoints_x,self.npoints_y])

    def coord_to_cell(self,pt):
        xcell=min(max(int( (pt[0]+self.resolution*self.npoints_x/2)/self.resolution),0),self.npoints_x-1)
        ycell=min(max(int( (pt[1]+self.resolution*self.npoints_y/2)/self.resolution),0),self.npoints_y-1)
        return np.array([xcell,ycell])

    def cell_to_coord(self,pt):
        xpt=(pt[0]-self.npoints_x/2)*self.resolution
        ypt=(pt[1]-self.npoints_y/2)*self.resolution
        return np.array([xpt,ypt])

    #works well for a single point, but I'm worried it is too big and too slow for many pts
    def cov_pt_toregion(self,pt,ncells=None,max_ncells=10):
        biggest_cov=np.max(abs(pt.covariance))
        if ncells==None:
            ncells=min(2*int(np.sqrt(biggest_cov)/self.resolution),max_ncells) #go out to two sigma
        center_cell=self.coord_to_cell(pt.vals)
        invcov=scipy.linalg.pinvh(pt.covariance)
        sigmult=np.sqrt(np.linalg.det(pt.covariance))
        detinvcov=np.linalg.det(invcov)
        ret=np.zeros([2*ncells+1,2*ncells+1])
        for i in range(-ncells,ncells+1):
            for j in range(-ncells,ncells+1):
                delta=self.cell_to_coord(np.array([i+center_cell[0],j+center_cell[1]]))-pt.vals
                nsigma=np.dot(delta,np.dot(invcov,delta)) #it keeps track of my transpose I think
                amount=(np.pi**2)*np.sqrt(detinvcov)*exp(-nsigma)*(self.resolution**2)
                ret[i+ncells,j+ncells]=amount
        return center_cell,ret

    def mark_pt_with_cov_blocked(self,pt): #given a point with covariance, mark out a blocked region
        #estimate what onesigma is in cells
        #print(pt.covariance)
        center_cell,area=self.cov_pt_toregion(pt)
        #print("max hit {}".format(np.max(area)))
        n=area.shape[0]
        for i in range(n):
            bigmap_i=i+center_cell[0]-int((n-1)/2)
            if bigmap_i<0 or bigmap_i>=self.npoints_x:
                continue
            for j in range(area.shape[0]):
                bigmap_j=j+center_cell[1]-int((n-1)/2)
                if bigmap_j<0 or bigmap_j>=self.npoints_y:
                    continue
                with self.gridmap_occupied_lock:
                    self.gridmap_occupied[bigmap_i,bigmap_j]+=area[i,j]
                    if self.gridmap_occupied[bigmap_i,bigmap_j]>self.max_counts:
                        self.gridmap_occupied[bigmap_i,bigmap_j]=self.max_counts

    def get_logprob_blocked(self,coord):
        with self.gridmap_occupied_lock:
            nb=self.gridmap_occupied[coord[0],coord[1]]
        #nc=self.gridmap_free[coord[0],coord[1]]
        #return np.log(nb/nc)
        return np.log(nb)

    def get_logprob_free(self,coord):
        with self.gridmap_occupied_lock:
            nb=self.gridmap_occupied[coord[0],coord[1]]
        nc=self.gridmap_free[coord[0],coord[1]]
        return np.log(nc/nb)

    def get_measurement_logprob(self,pose_vals,measurement):
        pts=measurement.to_xypoints_with_fixed_offset(pose_vals[0],pose_vals[1],pose_vals[2])
        sum_logprob=0
        for pt in pts:
            cell=self.coord_to_cell(pt.vals)
            sum_logprob+=self.get_logprob_blocked(cell)
        return sum_logprob

    def mark_explored(self,pose,lidarm):
        for i in range(len(lidarm)):
            A=self.coord_to_cell(pose.vals)
            B=self.coord_to_cell( [pose.vals[0]+lidarm.dists[i]*sin(lidarm.angles[i]-lidarm.angle_uncs[i]+pose.vals[2]),pose.vals[1]+lidarm.dists[i]*cos(lidarm.angles[i]-lidarm.angle_uncs[i]+pose.vals[2])])
            C=self.coord_to_cell( [pose.vals[0]+lidarm.dists[i]*sin(lidarm.angles[i]+lidarm.angle_uncs[i]+pose.vals[2]),pose.vals[1]+lidarm.dists[i]*cos(lidarm.angles[i]+lidarm.angle_uncs[i]+pose.vals[2])])
            explored_pts=triangle_inside(A,B,C)
            for p in explored_pts:
                if p[0]>=0 and p[0]<self.npoints_x and p[1]>0 and p[1]<self.npoints_y:
                    self.gridmap_explored[p[0],p[1]]+=1
                    if self.gridmap_explored[p[0],p[1]]>self.gridmap_explored_max:
                        self.gridmap_explored[p[0],p[1]]=self.gridmap_explored_max

    def guess_range_to_wall(self,pose,maxrange=3):
        occ_limit=20.0
        sum_occ=0
        A=self.coord_to_cell(pose.vals)
        Bfloat=[ pose.vals[0]+maxrange*sin(pose.vals[2]),pose.vals[1]+maxrange*cos(pose.vals[2])]
        B=self.coord_to_cell(Bfloat)
        points_on_way=bresenham(A,B)
        last_point=points_on_way[-1]
        with self.gridmap_occupied_lock:
            for p in points_on_way:
                sum_occ+=(self.gridmap_occupied[p[0],p[1]]-0.1)
                if sum_occ>occ_limit:
                    last_point=p
                    break
        #print("sum occ was {}".format(sum_occ))
        return np.linalg.norm(self.cell_to_coord(last_point)-np.array([pose.vals[0],pose.vals[1]]))


    def logsum_explored_in_cone(self,pose,range,theta1,theta2):
        A=self.coord_to_cell(pose.vals)
        Bfloat=[ pose.vals[0]+range*sin(pose.vals[2]+theta1),pose.vals[1]+range*cos(pose.vals[2]+theta1)]
        Cfloat=[ pose.vals[0]+range*sin(pose.vals[2]+theta2),pose.vals[1]+range*cos(pose.vals[2]+theta2)]
        B=self.coord_to_cell(Bfloat)
        C=self.coord_to_cell(Cfloat)
        pts=triangle_inside(A,B,C)
        logsum=0
        for p in pts:
            logsum+=np.log(self.gridmap_explored[ p[0],p[1]]+0.1)
        return logsum


    def delta_logsum_explored_in_cone(self,pose,range,theta1,theta2):
        A=self.coord_to_cell(pose.vals)
        Bfloat=[ pose.vals[0]+range*sin(pose.vals[2]+theta1),pose.vals[1]+range*cos(pose.vals[2]+theta1)]
        Cfloat=[ pose.vals[0]+range*sin(pose.vals[2]+theta2),pose.vals[1]+range*cos(pose.vals[2]+theta2)]
        B=self.coord_to_cell(Bfloat)
        C=self.coord_to_cell(Cfloat)
        pts=triangle_inside(A,B,C)
        logsum=0
        for p in pts:
            logsum+=1/(self.gridmap_explored[ p[0],p[1]]+0.1)-1/(self.gridmap_explored_max+0.1)
        return logsum

    #    Bfloat=[ pose.vals[0]+range*]
    #    explored_pts=triangle_inside(A,B,C)

#    def mark_explored(self,center_pt,points):
#            #TODO just a line if there's only one point
#            for i in range(len(points)-1):
#                A=self.coord_to_cell(center_pt.vals)
#                B=self.coord_to_cell(points[i].vals)
#                C=self.coord_to_cell(points[i+1].vals)
#                explored_pts=triangle_inside(A,B,C)
#                for p in explored_pts:
#                    if p[0]>=0 and p[0]<self.npoints_x and p[1]>0 and p[1]<self.npoints_y:
#                        self.gridmap_explored[p[0],p[1]]+=1


    def calculate_eval_map(self,center_vals,widths,measurement):
        npoints=20
        xs=np.linspace(center_vals[0]-widths[0]/2,center_vals[0]+widths[0]/2,npoints)
        ys=np.linspace(center_vals[1]-widths[1]/2,center_vals[1]+widths[1]/2,npoints)
        #print("xs {}".format(xs))
        #print("ys {}".format(ys))
        eval_map=np.zeros([npoints,npoints])
        for i in range(len(xs)):
            for j in range(len(ys)):
                eval_map[i,j]=self.get_measurement_logprob([xs[i],ys[j],center_vals[2]],measurement)
        return xs,ys,eval_map

    def eval_map_to_point(self,xs,ys,emap):
        sum_x=0
        sum_y=0
        sum_xx=0
        sum_xy=0
        sum_yy=0
        sum_p=0
        offset=np.min(emap) # we will arbitrarily set this to zero
        poffset=np.exp(offset)
        for i in range(emap.shape[0]):
            for j in range(emap.shape[1]):
                p=np.exp(emap[i,j])-poffset
                sum_p+=p
                sum_x+=p*xs[i]
                sum_y+=p*ys[j]
                sum_yy+=p*ys[j]*ys[j]
                sum_xx+=p*xs[i]*xs[i]
                sum_xy+=p*xs[i]*ys[j]
        xmean=sum_x/sum_p
        ymean=sum_y/sum_p
        xxmean=abs(sum_xx/sum_p-xmean*xmean)
        yymean=abs(sum_yy/sum_p-ymean*ymean)
        xymean=sum_xy/sum_p-xmean*ymean
        #TODO there is really no reason not to do this with angles too
        return BayesianArray(np.array([xmean,ymean,0]),np.array([[xxmean,xymean,0],[xymean,yymean,0],[0,0,1000]]))

    def occupancy_to_image(self,pose):
        with self.gridmap_occupied_lock:
            ret=self.gridmap_occupied.T[::-1,:]
            ret=255*ret/np.max(ret)
        map_image=cv.applyColorMap(ret.astype(np.uint8),cv.COLORMAP_HOT)
        center=self.coord_to_cell(pose.vals)
        startpt=(center[0],self.npoints_y-center[1]) #remember I have to flip the y axis
        arrowpt=self.coord_to_cell(pose.vals[0:2]+np.array([0.2*sin(pose.vals[2]),0.2*cos(pose.vals[2])]))
        cv.arrowedLine(map_image,startpt,(arrowpt[0],self.npoints_y-arrowpt[1]),(255,255,255),2, cv.LINE_AA, 0, 0.3)
        return cv.resize(map_image,(600,600))
