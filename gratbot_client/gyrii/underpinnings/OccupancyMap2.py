
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

    def to_exact(self):
        return ExactLidarMeasurement(self.dists,self.angles)

    def sample_to_exact(self):
        return ExactLidarMeasurement(np.random.normal(self.dists,self.dist_uncs),np.random.normal(self.angles,self.angle_uncs))

    def to_exact_downsample(self,ntotal):
        down=max(int(len(self.dists)/ntotal),1)
        mydists=self.dists[::down]
        myangles=self.angles[::down]
        return ExactLidarMeasurement(mydists,myangles)

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

    def exact_lidar_to_end_cells(self,exactpose,exactlidar):
        end_pts=exactlidar.to_xypoints(exactpose)
        end_cells=self.coords_to_cells(end_pts)
        return end_cells

    def exact_lidar_to_cells(self,exactpose,exactlidar):
        end_pts=exactlidar.to_xypoints(exactpose)
        start_cell=self.coord_to_cell(exactpose[0:2])
        end_cells=self.coords_to_cells(end_pts)
        all_cells=set()
        for e in end_cells:
            all_cells.update(bresenham(start_cell,e,returnset=True))
        end_cells_set=set(end_cells)
        free_cells=all_cells.difference(end_cells_set)
        return end_cells,free_cells

    def get_lidar_pose_map(self,exactpose,exactlidar,n_x,n_y):
        end_cells,free_cells=self.exact_lidar_to_cells(exactpose,exactlidar)
        xs=np.linspace(exactpose[0]-n_x*self.resolution,exactpose[0]+n_x*self.resolution,2*n_x+1)
        ys=np.linspace(exactpose[1]-n_y*self.resolution,exactpose[1]+n_y*self.resolution,2*n_y+1)
        ret=np.zeros([2*n_x+1,2*n_y+1])
        for i in range(-n_x,n_x+1):
            for j in range(-n_y,n_y+1):
                ret[i+n_x,j+n_y]=self.get_cell_score(free_cells,end_cells,i,j)
        return xs,ys,ret

    def get_lidar_pose_map_end_cells_only(self,exactpose,exactlidar,n_x,n_y,n_t):
        angle_resolution=0.017
        xs=np.linspace(exactpose[0]-n_x*self.resolution,exactpose[0]+n_x*self.resolution,2*n_x+1)
        ys=np.linspace(exactpose[1]-n_y*self.resolution,exactpose[1]+n_y*self.resolution,2*n_y+1)
        ts=np.linspace(exactpose[2]-n_t*angle_resolution,exactpose[2]+n_t*angle_resolution,2*n_t+1)
        ret=np.zeros([2*n_x+1,2*n_y+1,2*n_t+1])
        for k in range(-n_t,n_t+1):
            end_cells=self.exact_lidar_to_end_cells(exactpose+np.array([0,0,k*angle_resolution]),exactlidar)
            for i in range(-n_x,n_x+1):
                for j in range(-n_y,n_y+1):
                    ret[i+n_x,j+n_y,k+n_t]=self.get_cell_score_end_cells(end_cells,i,j)
        return xs,ys,ts,ret

    def pose_map_to_pose_prediction(self,xs,ys,posemap):
        sum_x=0
        sum_y=0
        sum_xx=0
        sum_xy=0
        sum_yy=0
        sum_p=0
        #offset=np.min(posemap) # we will arbitrarily set this to zero
        #poffset=np.exp(np.min(posemap))
        mymax=np.max(posemap)
        for i in range(posemap.shape[0]):
            for j in range(posemap.shape[1]):
                p=np.exp(posemap[i,j]-mymax)
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



    def pose_map_to_pose_prediction_with_angles(self,xs,ys,ts,posemap):
        sum_x=0
        sum_y=0
        sum_t=0
        sum_xx=0
        sum_xy=0
        sum_yy=0
        sum_tt=0
        sum_xt=0
        sum_yt=0
        sum_p=0
        #offset=np.min(posemap) # we will arbitrarily set this to zero
        #poffset=np.exp(np.min(posemap))
        mymax=np.max(posemap)
        for i in range(posemap.shape[0]):
            for j in range(posemap.shape[1]):
                for k in range(posemap.shape[2]):
                    p=np.exp(posemap[i,j,k]-mymax)
                    sum_p+=p
                    sum_x+=p*xs[i]
                    sum_y+=p*ys[j]
                    sum_t+=p*ts[k]
                    sum_yy+=p*ys[j]*ys[j]
                    sum_xx+=p*xs[i]*xs[i]
                    sum_xy+=p*xs[i]*ys[j]
                    sum_tt+=p*ts[k]*ts[k]
                    sum_xt+=p*xs[i]*ts[k]
                    sum_yt+=p*ys[j]*ts[k]
        xmean=sum_x/sum_p
        ymean=sum_y/sum_p
        tmean=sum_t/sum_p
        xxmean=abs(sum_xx/sum_p-xmean*xmean)
        yymean=abs(sum_yy/sum_p-ymean*ymean)
        ttmean=abs(sum_tt/sum_p-tmean*tmean)
        xymean=sum_xy/sum_p-xmean*ymean
        xtmean=sum_xt/sum_p-xmean*tmean
        ytmean=sum_yt/sum_p-ymean*tmean
        #TODO there is really no reason not to do this with angles too
        return BayesianArray(np.array([xmean,ymean,tmean]),np.array([[xxmean,xymean,xtmean],[xymean,yymean,ytmean],[xtmean,ytmean,ttmean]]))

    def get_cell_score_end_cells(self,end_cells,x_offset,y_offset):
        score=0
        with self.gridmap_occupied_lock:
            for cell in end_cells:
                #score+=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]

                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
                lp=self.gridmap_logodds[newx,newy]
                #score+=np.log( 1-1/(1+np.exp(lp)))
                if lp<-1:
                    score+=lp
                else:
                    score+=np.log( 1-1/(1+np.exp(lp)))
        return score

    def get_cell_score(self,free_cells,end_cells,x_offset,y_offset):
        score=0
        with self.gridmap_occupied_lock:
            for cell in free_cells:
                #score-=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
                lp=-self.gridmap_logodds[newx,newy]
                #score+=np.log( 1-1/(1+np.exp(lp)))
                #score-=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
                if lp<-1:
                    score+=lp
                else:
                    score+=np.log( 1-1/(1+np.exp(lp)))
            for cell in end_cells:
                #score+=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]

                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
                lp=self.gridmap_logodds[newx,newy]
                #score+=np.log( 1-1/(1+np.exp(lp)))
                if lp<-1:
                    score+=lp
                else:
                    score+=np.log( 1-1/(1+np.exp(lp)))
        return score

    def apply_exact_lidar_measurement(self,exactpose,exactlidar,weight=1):
        end_cells,free_cells=self.exact_lidar_to_cells(exactpose,exactlidar)
        with self.gridmap_occupied_lock:
            for cell in free_cells:
                self.gridmap_logodds[cell[0],cell[1]]=np.clip(self.gridmap_logodds[cell[0],cell[1]]+weight*self.free_logodds,-self.logodds_clip,self.logodds_clip)
            for cell in end_cells:
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

    def get_frontiers(self):
        #find all points that lay in boundary of known and unknown
        with self.gridmap_occupied_lock:
            unknown_mask=abs(self.gridmap_logodds)<0.5
            wall_mask=self.gridmap_logodds>0.5
            free_mask=self.gridmap_logodds<-0.5
        #ideally I want points that are unknown and adjacent to free cells but not adjacent to wall cells
        wall_mask_bleed=wall_mask | np.roll(wall_mask,1,axis=0) | np.roll(wall_mask,-1,axis=0) | np.roll(wall_mask,1,axis=1) | np.roll(wall_mask,-1,axis=1)
        free_mask_bleed=free_mask | np.roll(free_mask,1,axis=0) | np.roll(free_mask,-1,axis=0) | np.roll(free_mask,1,axis=1) | np.roll(free_mask,-1,axis=1)
        frontier= unknown_mask  & free_mask_bleed & np.logical_not(wall_mask_bleed)
        return frontier

    def frontier_to_image(self):
        frontiers=self.get_frontiers()
        ret=255*frontiers.T[::-1,:]
        map_image=cv.applyColorMap(ret.astype(np.uint8),cv.COLORMAP_HOT)
        return cv.resize(map_image,(600,600))
