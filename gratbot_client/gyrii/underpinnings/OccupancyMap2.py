
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
from scipy.signal import convolve

import threading
#for plotting
#from matplotlib import pyplot as plt
import cv2 as cv

class ExactLidarMeasurement:
    def __init__(self,dists,angles):
        self.dists=dists
        #self.angles=(np.array(angles) + np.pi) % (2 * np.pi) - np.pi
        self.angles=(np.array(angles) + 2*np.pi) % (2 * np.pi) #wraps to 0 to 2pi

    def downsample(self,sampleby):
        return ExactLidarMeasurement(self.dists[::sampleby],self.angles[::sampleby])

    def to_xypoints(self,exactpose): #exactpose [x,y,theta]
        xs=exactpose[0]*np.ones(len(self.angles))+self.dists*np.sin(self.angles+exactpose[2])
        ys=exactpose[1]*np.ones(len(self.angles))+self.dists*np.cos(self.angles+exactpose[2])
        return np.column_stack((xs,ys))

    def to_xypoints_extraone_with_downsample(self,ntotal): #
        down=max(int(len(self.dists)/ntotal),1)
        mydists=self.dists[::down]
        myangles=self.angles[::down]
        xs=mydists*np.sin(myangles)
        ys=mydists*np.cos(myangles)
        ones=np.ones(len(xs))
        return np.column_stack((xs,ys,ones))

    @staticmethod
    def from_xypoints(xypoints):
        dists=[]
        angles=[]
        for xy in xypoints:
            dists.append(np.linalg.norm(xy))
            angles.append(np.arctan2(xy[0],xy[1]))
        return ExactLidarMeasurement(dists,angles)

    def __len__(self):
        return len(self.dists)


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

    def get_entropy(self):
        entsum=0
        for du in self.dist_uncs:
            entsum+=np.log(du)
        return entsum

    def get_fit_gradient(self,m):
        #supposing self is a predicted scan from a map
        #and m is a measurement
        #return the chi-square of the measurement given the prediction
        #and the gradient
        #NOTE the angles had better be the same
        if len(m)!=len(self):
            raise "Error, not the same number of points"
        grad_sum=np.zeros(3)
        chisq=0
        for i in range(len(self)):
            if np.isnan(self.dists[i]) or isnan(m.dists[i]):
                continue
            dir=np.array([np.cos(m.angles[i]),np.sin(m.angles[i]),0])
            delta=(self.dists[i]-m.dists[i])/(self.dist_uncs[i])
            chisq+=delta*delta
            grad_sum+=2*delta/(self.dist_uncs[i])*dir
            nexti=(i+1)%(len(self))
            dangle=m.angles[nexti]-m.angles[i]
            dangle=(dangle + np.pi) % (2 * np.pi) - np.pi
            delta_turn=(self.dists[i]-m.dists[nexti])/(self.dist_uncs[i])
            grad_sum+=np.array([0,0,1])*(delta_turn**2-delta**2)/dangle
            #print("angle: {} measured: {} pred {}, delta {}, chisq {}".format(self.angles[i],m.dists[i],ufloat(self.dists[i],self.dist_uncs[i]),delta,delta*delta))
        return chisq,grad_sum


def apply_lidar_to_occupancy_map_with_precalc(dist_to_grid,theta_to_grid,lidar_angles,lidar_dists,alpha=0.05,beta=0.3,weight=1):
    total_free_mask=np.zeros(len(theta_to_grid),dtype=bool)
    total_occ_mask=np.zeros(len(theta_to_grid),dtype=bool)
    for i in range(len(lidar_dists)):
        r=lidar_dists[i]
        b=lidar_angles[i]
        angle_mask=(np.abs(theta_to_grid - b) <= beta/2.0)
        free_mask =  angle_mask & (dist_to_grid < (r - alpha/2.0))
        occ_mask = angle_mask & (np.abs(dist_to_grid - r) <= alpha/2.0)
        total_free_mask = total_free_mask | free_mask
        total_occ_mask = total_occ_mask | occ_mask
    return total_free_mask,total_occ_mask



def apply_lidar_to_occupancy_map(pose,grid_position_m,lidar_angles,lidar_dists,alpha=0.05,beta=0.3,weight=1):
    dx = grid_position_m.copy() # A tensor of coordinates of all cells
    dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
    dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
    #theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell
    theta_to_grid = np.arctan2(dx[0, :, :], dx[1, :, :]) - pose[2] # matrix of all bearings from robot to cell
    # Wrap to 0 to 2pi
    theta_to_grid[theta_to_grid < 0] += 2. * np.pi
    dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot
    return apply_lidar_to_occupancy_map_with_precalc(dist_to_grid,theta_to_grid,lidar_angles,lidar_dists,alpha,beta,weight)


#    total_free_mask=np.zeros(len(theta_to_grid),dtype=bool)
#    total_occ_mask=np.zeros(len(theta_to_grid),dtype=bool)
#    for i in range(len(exactlidar.dists)):
#        r=exactlidar.dists[i]
#        b=exactlidar.angles[i]
#        angle_mask=(np.abs(theta_to_grid - b) <= self.beta/2.0)
#        free_mask =  angle_mask & (dist_to_grid < (r - self.alpha/2.0))
#        occ_mask = angle_mask & (np.abs(dist_to_grid - r) <= self.alpha/2.0)
#        total_free_mask = total_free_mask | free_mask
#        total_occ_mask = total_occ_mask | occ_mask
#    self.gridmap_logodds[total_occ_mask]+=weight*self.occupied_logodds
#    self.gridmap_logodds[total_free_mask]+=weight*self.free_logodds


def index_logodds_map_cells(npoints_x,npoints_y,resolution):
        return np.array([np.tile(np.arange(-npoints_x*resolution/2, npoints_x*resolution/2, resolution)[:,None], (1, npoints_y)),np.tile(np.arange(-npoints_y*resolution/2, npoints_y*resolution/2, resolution)[:,None].T, (npoints_x, 1))])

class LocalizationException(Exception):
    def __init__(self,message):
        self.message=message

class OccupancyMap2:

    def __init__(self,resolution,npoints_x,npoints_y):
        self.resolution=resolution #grid size in meters
        self.npoints_x=npoints_x
        self.npoints_y=npoints_y
        self.gridmap_occupied_lock=threading.Lock()
        self.gridmap_logodds=np.zeros([self.npoints_x,self.npoints_y])
        #self.gridmap_logodds=-0.4*np.ones([self.npoints_x,self.npoints_y])
        self.occupied_logodds=0.8
        self.free_logodds=-0.2
        #self.logodds_clip=5
        self.logodds_clip=8
        #self.grid_position_m = np.array([np.tile(np.arange(-self.npoints_x*self.resolution/2, self.npoints_x*self.resolution/2, self.resolution)[:,None], (1, self.npoints_y)),np.tile(np.arange(-self.npoints_y*self.resolution/2, self.npoints_y*self.resolution/2, self.resolution)[:,None].T, (self.npoints_x, 1))])
        self.grid_position_m = index_logodds_map_cells(self.npoints_x,self.npoints_y,self.resolution)
        self.beta=np.radians(1.5)
        #self.beta=np.radians(2.0)
        self.alpha=resolution
        #print("grid pos {}".format(self.grid_position_m))


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

    #def apply_lidar_measurement(self,pose,mlidar,n_draws=10,weight=1):
    #    weight=weight/n_draws
    #    for i in range(n_draws):
    #        #exactpose=np.array(pose.vals)
    #        exactpose=pose.random_sample()
    #        #exactlidar=ExactLidarMeasurement(mlidar.dists,mlidar.angles)
    #        exactlidar=mlidar.sample_to_exact()
    #        self.apply_exact_lidar_measurement(exactpose,exactlidar,weight=weight)

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

    #def get_lidar_pose_map2(self,exactpose,exactlidar):
    #    xp=100
    #    res=self.resolution
    #    small_gridmap=np.zeros([xp,xp])
    #    grid_position_small = index_logodds_map_cells(xp,xp,res)
    #    total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(np.array([0,0,exactpose[2]]),grid_position_small,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta)
    #    small_gridmap[total_occ_mask]=1
    #    small_gridmap[total_free_mask]=-1
    #    #print(small_gridmap)
    #    #cv=np.convolve(small_gridmap,self.gridmap_logodds,mode='valid')
    #    #cv=convolve(small_gridmap,self.gridmap_logodds,mode='same')
    #    #cv=convolve(small_gridmap,self.gridmap_logodds,mode='valid')
    #    cv=convolve(self.gridmap_logodds,small_gridmap[::-1,::-1],mode='valid')
    #    return cv,np.linspace(exactpose[0]-self.resolution*xp/2,exactpose[0]+self.resolution*xp/2,xp),np.linspace(exactpose[1]-self.resolution*xp/2,exactpose[1]+self.resolution*xp/2,xp)


#    def get_lidar_pose_map3_withtheta(self,exactpose,exactlidar,theta_points=5,theta_res=np.radians(1)):
#        #xp=100
#        xp=150
#
#        res=self.resolution
#        small_gridmap=np.zeros([2*theta_points+1,xp,xp])
#        grid_position_small = index_logodds_map_cells(xp,xp,res)
#        total_points_array=[]
#        for i in range(-theta_points,theta_points+1,1):
#            dtheta=i*theta_res
#            total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(np.array([0,0,exactpose[2]+dtheta]),grid_position_small,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta)
#            small_gridmap[i+theta_points,total_occ_mask]=1
#            small_gridmap[i+theta_points,total_free_mask]=-1
#            total_points_array.append(np.count_nonzero(total_occ_mask)+np.count_nonzero(total_free_mask))
#        with self.gridmap_occupied_lock:
#            logodds_copy=self.gridmap_logodds[np.newaxis,:,:]
#            cv=convolve(logodds_copy,small_gridmap[:,::-1,::-1],mode='full')
#            cvabs=convolve(np.abs(logodds_copy),np.abs(small_gridmap[:,::-1,::-1]),mode='full')
#        oxp=cv.shape[1]
#        oyp=cv.shape[2]
#        otp=cv.shape[0]
#        return cv,np.linspace(-self.resolution*oxp/2,self.resolution*oxp/2,oxp),np.linspace(-self.resolution*oyp/2,self.resolution*oyp/2,oyp),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), cvabs
#
#    def get_lidar_pose_map2_withtheta(self,exactpose,exactlidar,theta_points=5,theta_res=np.radians(1)):
#        xp=100
#
#        res=self.resolution
#        small_gridmap=np.zeros([2*theta_points+1,xp,xp])
#        grid_position_small = index_logodds_map_cells(xp,xp,res)
#        total_points_array=[]
#        for i in range(-theta_points,theta_points+1,1):
#            dtheta=i*theta_res
#            total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(np.array([0,0,exactpose[2]+dtheta]),grid_position_small,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta)
#            small_gridmap[i+theta_points,total_occ_mask]=1
#            small_gridmap[i+theta_points,total_free_mask]=-1
#            total_points_array.append(np.count_nonzero(total_occ_mask)+np.count_nonzero(total_free_mask))
#        #print(small_gridmap)
#        #cv=np.convolve(small_gridmap,self.gridmap_logodds,mode='valid')
#        #cv=convolve(small_gridmap,self.gridmap_logodds,mode='same')
#        #cv=convolve(small_gridmap,self.gridmap_logodds,mode='valid')
#        logodds_copy=self.gridmap_logodds[np.newaxis,:,:]
#        #loglike_map=np.log( 1-1/(1+np.exp(logodds_copy)))
#        cv=convolve(logodds_copy,small_gridmap[:,::-1,::-1],mode='full')
#        #cv=convolve(loglike_map,small_gridmap[:,::-1,::-1],mode='full')
#        oxp=cv.shape[1]
#        oyp=cv.shape[2]
#        otp=cv.shape[0]
#        return cv,np.linspace(-self.resolution*oxp/2,self.resolution*oxp/2,oxp),np.linspace(-self.resolution*oyp/2,self.resolution*oyp/2,oyp),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), total_points_array
#        #return cv,np.linspace(exactpose[0]-self.resolution*oxp/2,exactpose[0]+self.resolution*oxp/2,oxp),np.linspace(exactpose[1]-self.resolution*oyp/2,exactpose[1]+self.resolution*oyp/2,oyp),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), total_points_array


#    def pose_map_to_pose_prediction2_with_theta_faster(self,posemap,xs,ys,ts,counts,center=None,spread_size_x=5,spread_size_y=5):
#        max_clip=8
#        spread_size_t=int((posemap.shape[0]-1)/2)
#        #posemap=np.exp(posemap/len(posemap))
#        if center is None:
#            best_point = np.unravel_index(np.argmax(posemap, axis=None), posemap.shape)
#            if best_point[0]==0 or best_point[0]>=posemap.shape[0]-1 or best_point[1]==0 or best_point[1]>=posemap.shape[1]-1 or best_point[2]==0 or best_point[2]>=posemap.shape[2]-1:
#                raise LocalizationException("Best point on edge of map")
#        else:
#            a=ts.searchsorted(center.vals[2])
#            b=xs.searchsorted(center.vals[0])
#            c=ys.searchsorted(center.vals[1])
#
#        #sigma=np.sqrt(np.max(counts))*np.sqrt(2*max_clip)
#        sigma=np.sqrt(np.max(counts))*np.sqrt(max_clip)
#
#        imin=max(0,-spread_size_t+best_point[0])
#        imax=min(spread_size_t+1+best_point[0],posemap.shape[0])
#        jmin=max(0,-spread_size_x+best_point[1])
#        jmax=min(spread_size_x+1+best_point[1],posemap.shape[1])
#        kmin=max(0,-spread_size_y+best_point[2])
#        kmax=min(spread_size_y+1+best_point[2],posemap.shape[2])
#
#        submap=posemap[imin:imax,jmin:jmax,kmin:kmax]
#        probmap=np.exp((submap-np.max(submap))/sigma)
#
#        ybox=np.tile(ys[kmin:kmax],(probmap.shape[0],probmap.shape[1],1))
#        tbox=np.reshape(np.repeat(ts[imin:imax],probmap.shape[1]*probmap.shape[2]),probmap.shape)
#        xbox_one=np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]))
#        xbox=np.tile(xbox_one,(probmap.shape[0],1,1))
#        #xbox=np.tile(np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]),(probmap.shape[0],1,1)))
#
#        sum_p=np.sum(probmap)
#        sum_x=np.sum(probmap*xbox)
#        sum_y=np.sum(probmap*ybox)
#        sum_t=np.sum(probmap*tbox)
#        sum_xx=np.sum(probmap*xbox*xbox)
#        sum_xy=np.sum(probmap*xbox*ybox)
#        sum_yy=np.sum(probmap*ybox*ybox)
#        sum_tt=np.sum(probmap*tbox*tbox)
#        sum_xt=np.sum(probmap*xbox*tbox)
#        sum_yt=np.sum(probmap*ybox*tbox)
#
#        xmean=sum_x/sum_p
#        ymean=sum_y/sum_p
#        tmean=sum_t/sum_p
#        xxmean=abs(sum_xx/sum_p-xmean*xmean)
#        yymean=abs(sum_yy/sum_p-ymean*ymean)
#        ttmean=abs(sum_tt/sum_p-tmean*tmean)
#        xymean=sum_xy/sum_p-xmean*ymean
#        xtmean=sum_xt/sum_p-xmean*tmean
#        ytmean=sum_yt/sum_p-ymean*tmean
#        #Never return better than the half resolution
#        xres=xs[1]-xs[0]
#        yres=ys[1]-ys[0]
#        tres=ts[1]-ts[0]
#        xxmean+=xres*xres/4
#        yymean+=yres*yres/4
#        ttmean+=tres*tres/4
#        #So because of some extra minus of the convolution, x and y and t are flipped here
#        return BayesianArray(np.array([xmean,ymean,tmean]),np.array([[xxmean,xymean,xtmean],[xymean,yymean,ytmean],[xtmean,ytmean,ttmean]]))
#

#    def pose_map_to_pose_prediction3_with_theta_faster(self,posemap,xs,ys,ts,counts,center=None,spread_size_x=5,spread_size_y=5,supress_errors=False):
#        #max_clip=8
#        spread_size_t=int((posemap.shape[0]-1)/2)
#        #posemap=np.exp(posemap/len(posemap))
#        #if center is None:
#        #    best_point = np.unravel_index(np.argmax(posemap, axis=None), posemap.shape)
#        #    if best_point[0]==0 or best_point[0]>=posemap.shape[0]-1 or best_point[1]==0 or best_point[1]>=posemap.shape[1]-1 or best_point[2]==0 or best_point[2]>=posemap.shape[2]-1:
#        #        if not supress_errors:
#        #            raise LocalizationException("Best point on edge of map")
#        #else:
#        if center is None:
#            a=int(len(ts)/2)
#            b=int(len(xs)/2)
#            c=int(len(ys)/2)
#        else:
#            a=ts.searchsorted(center.vals[2])
#            b=xs.searchsorted(center.vals[0])
#            c=ys.searchsorted(center.vals[1])
#        best_point=(a,b,c)
#        if best_point[0]==0 or best_point[0]>=posemap.shape[0]-1 or best_point[1]==0 or best_point[1]>=posemap.shape[1]-1 or best_point[2]==0 or best_point[2]>=posemap.shape[2]-1:
#            if not supress_errors:
#                raise LocalizationException("centerpoint on edge of map")
#
#        #sigma=np.sqrt(np.max(counts))*np.sqrt(2*max_clip)
#        #sigma=np.sqrt(np.max(counts))*np.sqrt(max_clip)
#
#        imin=max(0,-spread_size_t+best_point[0])
#        imax=min(spread_size_t+1+best_point[0],posemap.shape[0])
#        jmin=max(0,-spread_size_x+best_point[1])
#        jmax=min(spread_size_x+1+best_point[1],posemap.shape[1])
#        kmin=max(0,-spread_size_y+best_point[2])
#        kmax=min(spread_size_y+1+best_point[2],posemap.shape[2])
#
#        submap=posemap[imin:imax,jmin:jmax,kmin:kmax]
#        submap_counts=counts[imin:imax,jmin:jmax,kmin:kmax]
#        submap_counts_cutoff=800 #this is about 100 points overlap
#        invalid_bins=submap_counts<submap_counts_cutoff
#        #maxval=np.max(submap)
#        logprobmap=(submap-submap_counts)/(np.sqrt(submap_counts+submap_counts_cutoff))
#        probmap=np.exp(logprobmap)
#        probmap[invalid_bins]=0
#        real_best_point = np.unravel_index(np.argmax(probmap, axis=None), logprobmap.shape)
#        if not ( (0<real_best_point[0]<logprobmap.shape[0]-1) and (0<real_best_point[1]<logprobmap.shape[1]-1) and (0<real_best_point[2]<logprobmap.shape[2]-1) ):
#            raise LocalizationException("Best point on edge of map")
#
#        #probmap=np.exp((submap-maxval)/(np.sqrt(submap_counts+1)))
#        #TODO HERE IS THE DEAL I THINK MAXVAL IS NOT THE RIGHT THING TO SUBTRACT
#
#        max_prob_ind = np.unravel_index(np.argmax(probmap, axis=None), probmap.shape)
#        center_score=logprobmap[max_prob_ind]
#
#        ybox=np.tile(ys[kmin:kmax],(probmap.shape[0],probmap.shape[1],1))
#        tbox=np.reshape(np.repeat(ts[imin:imax],probmap.shape[1]*probmap.shape[2]),probmap.shape)
#        xbox_one=np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]))
#        xbox=np.tile(xbox_one,(probmap.shape[0],1,1))
#        #xbox=np.tile(np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]),(probmap.shape[0],1,1)))
#
#        sum_p=np.sum(probmap)
#        sum_x=np.sum(probmap*xbox)
#        sum_y=np.sum(probmap*ybox)
#        sum_t=np.sum(probmap*tbox)
#        sum_xx=np.sum(probmap*xbox*xbox)
#        sum_xy=np.sum(probmap*xbox*ybox)
#        sum_yy=np.sum(probmap*ybox*ybox)
#        sum_tt=np.sum(probmap*tbox*tbox)
#        sum_xt=np.sum(probmap*xbox*tbox)
#        sum_yt=np.sum(probmap*ybox*tbox)
#
#        xmean=sum_x/sum_p
#        ymean=sum_y/sum_p
#        tmean=sum_t/sum_p
#        xxmean=abs(sum_xx/sum_p-xmean*xmean)
#        yymean=abs(sum_yy/sum_p-ymean*ymean)
#        ttmean=abs(sum_tt/sum_p-tmean*tmean)
#        xymean=sum_xy/sum_p-xmean*ymean
#        xtmean=sum_xt/sum_p-xmean*tmean
#        ytmean=sum_yt/sum_p-ymean*tmean
#        #Never return better than the half resolution
#        xres=xs[1]-xs[0]
#        yres=ys[1]-ys[0]
#        tres=ts[1]-ts[0]
#        xxmean+=xres*xres/4
#        yymean+=yres*yres/4
#        ttmean+=tres*tres/4
#        #So because of some extra minus of the convolution, x and y and t are flipped here
#        return BayesianArray(np.array([xmean,ymean,tmean]),np.array([[xxmean,xymean,xtmean],[xymean,yymean,ytmean],[xtmean,ytmean,ttmean]])),center_score
#
#
#    def get_lidar_pose_map4_withtheta(self,exactpose,exactlidar,theta_points=5,theta_res=np.radians(1)):
#        #xp=100
#        xp=150
#
#        res=self.resolution
#        small_gridmap=np.zeros([2*theta_points+1,xp,xp])
#        grid_position_small = index_logodds_map_cells(xp,xp,res)
#        total_points_array=[]
#        for i in range(-theta_points,theta_points+1,1):
#            dtheta=i*theta_res
#            total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(np.array([0,0,exactpose[2]+dtheta]),grid_position_small,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta)
#            small_gridmap[i+theta_points,total_occ_mask]=1
#            small_gridmap[i+theta_points,total_free_mask]=-1
#            total_points_array.append(np.count_nonzero(total_occ_mask)+np.count_nonzero(total_free_mask))
#        with self.gridmap_occupied_lock:
#            #logodds_copy=self.gridmap_logodds[np.newaxis,:,:]
#            logodds_copy=self.gridmap_logodds
#            cv=np.zeros([small_gridmap.shape[0],logodds_copy.shape[0],logodds_copy.shape[1]])
#            cvabs=np.zeros([small_gridmap.shape[0],logodds_copy.shape[0],logodds_copy.shape[1]])
#            for i in range(small_gridmap.shape[0]):
#                #print(logodds_copy.shape)
#                #print(small_gridmap.shape[0])
#                cv[i]=convolve(logodds_copy,small_gridmap[i,::-1,::-1],mode='same')
#                #cvabs[i]=convolve(np.abs(logodds_copy),np.abs(small_gridmap[i,::-1,::-1]),mode='same')
#
#
#            #cv=convolve(logodds_copy,small_gridmap[:,::-1,::-1],mode='full')
#            #cv=convolve(logodds_copy,small_gridmap[::-1,::-1,::-1],mode='')
#            #cv=convolve(small_gridmap[::-1,::-1,::-1],logodds_copy,mode='same')
#            #cvabs=convolve(np.abs(logodds_copy),np.abs(small_gridmap[:,::-1,::-1]),mode='full')
#        cvabs=1
#        oxp=cv.shape[1]
#        oyp=cv.shape[2]
#        otp=cv.shape[0]
#        return cv,np.linspace(-self.resolution*oxp/2,self.resolution*oxp/2,oxp),np.linspace(-self.resolution*oyp/2,self.resolution*oyp/2,oyp),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), cvabs
#

    def get_lidar_pose_map5_withtheta(self,exactpose,exactlidar,theta_points=5,theta_res=np.radians(1),local_lidar_map_size=150,global_map_halfsize=100):
        #xp=100
        xp=local_lidar_map_size
        #if 2*global_map_halfsize < local_map_size, then this really won't work
        #it's only valid for positions within global_map_halfsize/2-local_map_size

        map_spread=global_map_halfsize #plus or minus this much in any direction
        map_center=self.coord_to_cell(exactpose[0:2])
        gridlen=self.gridmap_logodds.shape[0]

        res=self.resolution
        small_gridmap=np.zeros([2*theta_points+1,xp,xp])
        grid_position_small = index_logodds_map_cells(xp,xp,res)
        total_points_array=[]
        for i in range(-theta_points,theta_points+1,1):
            dtheta=i*theta_res
            total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(np.array([0,0,exactpose[2]+dtheta]),grid_position_small,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta)
            small_gridmap[i+theta_points,total_occ_mask]=1
            small_gridmap[i+theta_points,total_free_mask]=-1
            total_points_array.append(np.count_nonzero(total_occ_mask)+np.count_nonzero(total_free_mask))
        with self.gridmap_occupied_lock:
            #logodds_copy=self.gridmap_logodds[np.newaxis,:,:]
            #print("map spread {}".format(map_spread))
            #print("{}:{},{}:{}".format(map_center[0]-map_spread,map_center[0]+map_spread,map_center[1]-map_spread,map_center[1]+map_spread))
            logodds_copy=self.gridmap_logodds[map_center[0]-map_spread:map_center[0]+map_spread,map_center[1]-map_spread:map_center[1]+map_spread]
            cv=np.zeros([small_gridmap.shape[0],logodds_copy.shape[0],logodds_copy.shape[1]])
            #cvabs=np.zeros([small_gridmap.shape[0],logodds_copy.shape[0],logodds_copy.shape[1]])
            for i in range(small_gridmap.shape[0]):
                #print(logodds_copy.shape)
                #print(small_gridmap.shape[0])
                cv[i]=convolve(logodds_copy,small_gridmap[i,::-1,::-1],mode='same')
                #cvabs[i]=convolve(np.abs(logodds_copy),np.abs(small_gridmap[i,::-1,::-1]),mode='same')


            #cv=convolve(logodds_copy,small_gridmap[:,::-1,::-1],mode='full')
            #cv=convolve(logodds_copy,small_gridmap[::-1,::-1,::-1],mode='')
            #cv=convolve(small_gridmap[::-1,::-1,::-1],logodds_copy,mode='same')
            #cvabs=convolve(np.abs(logodds_copy),np.abs(small_gridmap[:,::-1,::-1]),mode='full')
        cvabs=1
        oxp=cv.shape[1]
        oyp=cv.shape[2]
        otp=cv.shape[0]
        #return cv,np.linspace(-self.resolution*oxp/2,self.resolution*oxp/2,oxp),np.linspace(-self.resolution*oyp/2,self.resolution*oyp/2,oyp),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), cvabs
        return cv,np.linspace(self.resolution*(map_center[0]-map_spread-gridlen/2),self.resolution*(map_center[0]+map_spread-gridlen/2),2*map_spread),np.linspace(self.resolution*(map_center[1]-map_spread-gridlen/2),self.resolution*(map_center[1]+map_spread-gridlen/2),2*map_spread),np.linspace(exactpose[2]-theta_points*theta_res,exactpose[2]+theta_points*theta_res,otp), cvabs

    def pose_map_to_pose_prediction4_with_theta_faster(self,posemap,xs,ys,ts,counts,center=None,spread_size_x=5,spread_size_y=5,supress_errors=False):
        spread_size_t=int((posemap.shape[0]-1)/2)
        if center is None:
            a=int(len(ts)/2)
            b=int(len(xs)/2)
            c=int(len(ys)/2)
        else:
            a=ts.searchsorted(center.vals[2])
            b=xs.searchsorted(center.vals[0])
            c=ys.searchsorted(center.vals[1])
        best_point=(a,b,c)
        if best_point[0]==0 or best_point[0]>=posemap.shape[0]-1 or best_point[1]==0 or best_point[1]>=posemap.shape[1]-1 or best_point[2]==0 or best_point[2]>=posemap.shape[2]-1:
            if not supress_errors:
                raise LocalizationException("centerpoint on edge of map")

        #sigma=np.sqrt(np.max(counts))*np.sqrt(2*max_clip)
        #sigma=np.sqrt(np.max(counts))*np.sqrt(max_clip)

        imin=max(0,-spread_size_t+best_point[0])
        imax=min(spread_size_t+1+best_point[0],posemap.shape[0])
        jmin=max(0,-spread_size_x+best_point[1])
        jmax=min(spread_size_x+1+best_point[1],posemap.shape[1])
        kmin=max(0,-spread_size_y+best_point[2])
        kmax=min(spread_size_y+1+best_point[2],posemap.shape[2])

        submap=posemap[imin:imax,jmin:jmax,kmin:kmax]
        clipval=8
        themax=np.max(posemap)
        numrightmap=(submap+themax)/clipval
        numwrongmap=np.max(numrightmap)-numrightmap
        p=0.6
        logprobmap=numrightmap*np.log(p)+numwrongmap*np.log(1-p)
        logprobmap-=np.max(logprobmap)
        probmap=np.exp(logprobmap)

        real_best_point = np.unravel_index(np.argmax(probmap, axis=None), logprobmap.shape)
        if not ( (0<real_best_point[0]<logprobmap.shape[0]-1) and (0<real_best_point[1]<logprobmap.shape[1]-1) and (0<real_best_point[2]<logprobmap.shape[2]-1) ):
            if not supress_errors:
                raise LocalizationException("Best point on edge of map: {} out of {}".format(real_best_point,logprobmap.shape))

        max_prob_ind = np.unravel_index(np.argmax(probmap, axis=None), probmap.shape)
        center_score=logprobmap[max_prob_ind]

        ybox=np.tile(ys[kmin:kmax],(probmap.shape[0],probmap.shape[1],1))
        tbox=np.reshape(np.repeat(ts[imin:imax],probmap.shape[1]*probmap.shape[2]),probmap.shape)
        xbox_one=np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]))
        xbox=np.tile(xbox_one,(probmap.shape[0],1,1))
        #xbox=np.tile(np.reshape(np.repeat(xs[jmin:jmax],probmap.shape[2]),(probmap.shape[1],probmap.shape[2]),(probmap.shape[0],1,1)))

        sum_p=np.sum(probmap)
        sum_x=np.sum(probmap*xbox)
        sum_y=np.sum(probmap*ybox)
        sum_t=np.sum(probmap*tbox)
        sum_xx=np.sum(probmap*xbox*xbox)
        sum_xy=np.sum(probmap*xbox*ybox)
        sum_yy=np.sum(probmap*ybox*ybox)
        sum_tt=np.sum(probmap*tbox*tbox)
        sum_xt=np.sum(probmap*xbox*tbox)
        sum_yt=np.sum(probmap*ybox*tbox)

        xmean=sum_x/sum_p
        ymean=sum_y/sum_p
        tmean=sum_t/sum_p
        xxmean=abs(sum_xx/sum_p-xmean*xmean)
        yymean=abs(sum_yy/sum_p-ymean*ymean)
        ttmean=abs(sum_tt/sum_p-tmean*tmean)
        xymean=sum_xy/sum_p-xmean*ymean
        xtmean=sum_xt/sum_p-xmean*tmean
        ytmean=sum_yt/sum_p-ymean*tmean
        #Never return better than the half resolution
        xres=xs[1]-xs[0]
        yres=ys[1]-ys[0]
        tres=ts[1]-ts[0]
        xxmean+=xres*xres/4
        yymean+=yres*yres/4
        ttmean+=tres*tres/4
        #So because of some extra minus of the convolution, x and y and t are flipped here
        return BayesianArray(np.array([xmean,ymean,tmean]),np.array([[xxmean,xymean,xtmean],[xymean,yymean,ytmean],[xtmean,ytmean,ttmean]])),center_score


#    def get_lidar_pose_map(self,exactpose,exactlidar,n_x,n_y):
#        end_cells,free_cells=self.exact_lidar_to_cells(exactpose,exactlidar)
#        xs=np.linspace(exactpose[0]-n_x*self.resolution,exactpose[0]+n_x*self.resolution,2*n_x+1)
#        ys=np.linspace(exactpose[1]-n_y*self.resolution,exactpose[1]+n_y*self.resolution,2*n_y+1)
#        ret=np.zeros([2*n_x+1,2*n_y+1])
#        for i in range(-n_x,n_x+1):
#            for j in range(-n_y,n_y+1):
#                ret[i+n_x,j+n_y]=self.get_cell_score(free_cells,end_cells,i,j)
#        return xs,ys,ret
#
#    def get_lidar_pose_map_end_cells_only(self,exactpose,exactlidar,n_x,n_y,n_t):
#        angle_resolution=0.017
#        xs=np.linspace(exactpose[0]-n_x*self.resolution,exactpose[0]+n_x*self.resolution,2*n_x+1)
#        ys=np.linspace(exactpose[1]-n_y*self.resolution,exactpose[1]+n_y*self.resolution,2*n_y+1)
#        ts=np.linspace(exactpose[2]-n_t*angle_resolution,exactpose[2]+n_t*angle_resolution,2*n_t+1)
#        ret=np.zeros([2*n_x+1,2*n_y+1,2*n_t+1])
#        for k in range(-n_t,n_t+1):
#            end_cells=self.exact_lidar_to_end_cells(exactpose+np.array([0,0,k*angle_resolution]),exactlidar)
#            for i in range(-n_x,n_x+1):
#                for j in range(-n_y,n_y+1):
#                    ret[i+n_x,j+n_y,k+n_t]=self.get_cell_score_end_cells(end_cells,i,j)
#        return xs,ys,ts,ret

#    def get_cell_score_end_cells(self,end_cells,x_offset,y_offset):
#        score=0
#        with self.gridmap_occupied_lock:
#            for cell in end_cells:
#                #score+=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
#
#                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
#                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
#                lp=self.gridmap_logodds[newx,newy]
#                #score+=np.log( 1-1/(1+np.exp(lp)))
#                if lp<-1:
#                    score+=lp
#                else:
#                    score+=np.log( 1-1/(1+np.exp(lp)))
#        return score
#
#    def get_cell_score(self,free_cells,end_cells,x_offset,y_offset):
#        score=0
#        with self.gridmap_occupied_lock:
#            for cell in free_cells:
#                #score-=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
#                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
#                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
#                lp=-self.gridmap_logodds[newx,newy]
#                #score+=np.log( 1-1/(1+np.exp(lp)))
#                #score-=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
#                if lp<-1:
#                    score+=lp
#                else:
#                    score+=np.log( 1-1/(1+np.exp(lp)))
#            for cell in end_cells:
#                #score+=self.gridmap_logodds[cell[0]+x_offset,cell[1]+y_offset]
#
#                newx=max(0,min(cell[0]+x_offset,self.npoints_x-1))
#                newy=max(0,min(cell[1]+y_offset,self.npoints_y-1))
#                lp=self.gridmap_logodds[newx,newy]
#                #score+=np.log( 1-1/(1+np.exp(lp)))
#                if lp<-1:
#                    score+=lp
#                else:
#                    score+=np.log( 1-1/(1+np.exp(lp)))
#        return score



    def apply_exact_lidar_measurement3(self,exactpose,exactlidar,weight=1):
        total_free_mask,total_occ_mask=apply_lidar_to_occupancy_map(exactpose,self.grid_position_m,exactlidar.angles,exactlidar.dists,alpha=self.alpha,beta=self.beta,weight=weight)
        with self.gridmap_occupied_lock:
            self.gridmap_logodds[total_occ_mask]+=weight*self.occupied_logodds
            self.gridmap_logodds[total_free_mask]+=weight*self.free_logodds
            self.gridmap_logodds=np.clip(self.gridmap_logodds,-self.logodds_clip,self.logodds_clip)

    def occupancy_to_image(self,pose):
        with self.gridmap_occupied_lock:
            ret=np.exp(self.gridmap_logodds.T[::-1,:])
        ret=255*(np.ones(ret.shape)-np.ones(ret.shape)/(np.ones(ret.shape)+ret))
        map_image=cv.applyColorMap(ret.astype(np.uint8),cv.COLORMAP_HOT)
        center=self.coord_to_cell(pose.vals)
        startpt=(center[0],self.npoints_y-center[1]) #remember I have to flip the y axis
        arrow_length=0.5
        arrowpt=self.coord_to_cell(pose.vals[0:2]+np.array([arrow_length*sin(pose.vals[2]),arrow_length*cos(pose.vals[2])]))
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

    def get_closest_frontier(self,center,min_contig_points=2):
        frontiers=self.get_frontiers()
        indices=np.mask_indices(frontiers)
        dists=[np.linalg.norm(self.cell_to_coord(ind)-center) for ind in indices]
        sorted_dists,sorted_indices=zip(*sorted(zip(dists,indices)))
        return sorted_indices[0] #untestend an not sure itdoes what I want


    def frontier_to_image(self):
        frontiers=self.get_frontiers()
        ret=255*frontiers.T[::-1,:]
        map_image=cv.applyColorMap(ret.astype(np.uint8),cv.COLORMAP_HOT)
        return cv.resize(map_image,(600,600))


    def predict_distance(self,start_position,angle,max_dist=2.0,on_p_cut=0.999,loud=False):
        center=self.coord_to_cell(start_position[0:2])
        end=self.coord_to_cell(start_position[0:2]+max_dist*np.array([sin(angle),cos(angle)]))
        cells_on_way=bresenham(center,end)
        on_p=1
        sum_p=0
        sum_x=0
        sum_xx=0
        for i in range(len(cells_on_way)):
            lp=self.gridmap_logodds[cells_on_way[i][0],cells_on_way[i][1]]
            p=1-1/(1+np.exp(lp))
            d=np.linalg.norm(self.cell_to_coord(cells_on_way[i])-start_position)
            if loud:
                print("onp {} p {} d {}".format(on_p,p,d))
            sum_p+=p
            sum_x+=p*d
            sum_xx+=p*d*d
            on_p=on_p*(1-p)
            if on_p<(1-on_p_cut):
                break
        return sum_x/sum_p,np.sqrt(sum_xx/sum_p-(sum_x/sum_p)**2+self.resolution*self.resolution) #never claim better than one pixel resolution


    def raytrace_through_free(self,start_position,angle,max_dist=3.0,blocked_threshhold=2,free_threshhold=-2,breakthrough=3):
        #trace a ray from start position and angle through free nodes, stop when it hits blocked or unknown
        center=self.coord_to_cell(start_position[0:2])
        end=self.coord_to_cell(start_position[0:2]+max_dist*np.array([sin(angle),cos(angle)]))
        cells_on_way=bresenham(center,end)
        unknown_counter=0
        for i in range(len(cells_on_way)):
            lp=self.gridmap_logodds[cells_on_way[i][0],cells_on_way[i][1]]
            if lp>blocked_threshhold:
                return np.linalg.norm(self.cell_to_coord(cells_on_way[i])-start_position),"blocked"
            if lp>free_threshhold:
                unknown_counter=unknown_counter+1
                if unknown_counter>=breakthrough:
                    return np.linalg.norm(self.cell_to_coord(cells_on_way[i])-start_position),"unknown"
        return np.linalg.norm(self.cell_to_coord(cells_on_way[-1])-start_position),"free"

    def predict_distance_with_threshhold(self,start_position,angle,max_dist=3.0,threshhold=0.5,base_unc=0.05):
        #print("angle {}".format(angle))
        center=self.coord_to_cell(start_position[0:2])
        #end=self.coord_to_cell(start_position[0:2]+max_dist*np.array([sin(angle),cos(angle)]))
        end=self.coord_to_cell(start_position[0:2]+max_dist*np.array([cos(angle),sin(angle)]))
        cells_on_way=bresenham(center,end)
        with self.gridmap_occupied_lock:
            for i in range(len(cells_on_way)):
    #            if cells_on_way[i][0]<0 or cells_on_way[i][0]>self.gridmap_logodds.shape[0] or cells_on_way[i][1]<0 or cells_on_way[i][1]>self.gridmap_logodds.shape[1]:
    #                break
                lp=self.gridmap_logodds[cells_on_way[i][0],cells_on_way[i][1]]
                #print("at {} lp is {}".format(cells_on_way[i],lp))
                if lp>threshhold:
                    return np.linalg.norm(self.cell_to_coord(cells_on_way[i])-start_position),base_unc
        return max_dist,0
        #return np.NAN,np.NAN

    def predict_distance2(self,start_position,angle,max_dist=2.0,on_p_cut=0.99):
        #this one is more statistically correct, but since I ignored correlation when creating the map
        #I think it goes wrong to do it now
        center=self.coord_to_cell(start_position[0:2])
        end=self.coord_to_cell(start_position[0:2]+max_dist*np.array([sin(angle),cos(angle)]))
        cells_on_way=bresenham(center,end)
        on_p=1
        sum_x=0
        sum_xx=0
        for i in range(len(cells_on_way)):
            lp=self.gridmap_logodds[cells_on_way[i][0],cells_on_way[i][1]]
            p=1-1/(1+np.exp(lp))
            d=np.linalg.norm(self.cell_to_coord(cells_on_way[i])-start_position)
            print("onp {} p {} d {}".format(on_p,p,d))
            sum_x+=on_p*p*d
            sum_xx+=on_p*p*d*d
            on_p=on_p*(1-p)
            if on_p<(1-on_p_cut):
                break
        return sum_x,np.sqrt(sum_xx-sum_x*sum_x)

    def predict_scan_from_angles(self,start_position,angles,max_dist=4.0):
        angle_offset=0
        if len(start_position)==3:
            angle_offset=start_position[2]
            start_position=start_position[0:2]
        angle_unc=np.zeros((len(angles)))
        dists=[]
        dist_uncs=[]
        for angle in angles:
            #d,dd=self.predict_distance_with_threshhold(start_position,angle+angle_offset,max_dist)
            d,dd=self.predict_distance(start_position,angle+angle_offset,max_dist)
            dists.append(d)
            dist_uncs.append(dd)
        return LidarMeasurement(dists,dist_uncs,angles,angle_unc)

    def predict_scan(self,start_position,n_points,max_dist=2.0):
        angles=np.linspace(0,2*np.pi,n_points)
        return self.predict_scan_from_angles(start_position,angles,max_dist)


    def save_to_file(self,fname):
        with open(fname,"w") as f:
            np.save(fname,self.gridmap_logodds)

    @staticmethod
    def load_from_file(fname,res=0.05):
        with open(fname,"rb") as f:
            gridmap_logodds=np.load(f)
            ret=OccupancyMap2(res,gridmap_logodds.shape[0],gridmap_logodds.shape[1])
            ret.gridmap_logodds=np.clip(gridmap_logodds,-ret.logodds_clip,ret.logodds_clip)
            return ret


    #Generate a local map
    #Convolve local map on global map
