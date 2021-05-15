#gridmap to abstract some of my occupancy map functions

#this is a 2d map with a bunch of precaclulated info about references to the center
class GridMap:
    def __init__(self,resolution=0.05,xysize=100,theta_bin_size=):
        self.resolution=resolution #how many meters per square
        self.xy_size=xysize #how many squares per side
        self.data=np.zeros([self.xy_size,self.xy_size])
        self.cell_indices=np.mgrid[-self.resolution*self.xy_size/2:self.resolution*self.xy_size/2:1j*self.xy_size,-self.resolution*self.xy_size/2:self.resolution*self.xy_size/2:1j*self.xy_size]
