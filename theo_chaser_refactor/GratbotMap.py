import numpy as np
#
import cv2 as cv

class GratbotMap():
    def __init__(self):
        self.square_edge_length=0.1 #in meters
        self.side_size=100
        self.map_marker_cleared=np.zeros([self.side_size,self.side_size])
        self.map_marker_blocked=np.zeros([self.side_size,self.side_size])
        #0,0, is in the center

        #self.kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        self.pose=np.zeros(3) #x y angle
        self.pose_covariance=1e-4*np.identity(3) #x,y,angle
        self.pose_covariance[2][2]=np.pi*np.pi # idon't know starting angle
        self.pose_min_covariance=1e-4*np.identity(3) #this would be called process noise in a kalman filter
        self.last_time=0
        self.heading_offset=0
        self.sigma_too_big=1e10

    def update_pose_with_measurement(self,x_vec,x_cov): #x,y,angle and covariance
        #use this if I made a measurement of the pose, not a change
        mu_1=x_vec
        mu_2=self.pose
        sigma_1=x_cov
        sigma_2=self.pose_covariance
        sumsigmainv=np.linalg.inv(sigma_1+sigma_2)
        mu_new=np.dot(sigma_2,np.dot(sumsigmainv,mu_1))+np.dot(sigma_1,np.dot(sumsigmainv,mu_2))
        #print("mu new {}".format(mu_new))
        sigma_new=np.dot(sigma_1,np.dot(sumsigmainv,sigma_2))+self.pose_min_covariance
        self.pose=mu_new
        self.pose_covariance=sigma_new

    def change_pose_with_offset(self,x_vec,x_cov):
        #use this if I made a -change- to the pose, not a measurement
        self.pose+=x_vec
        self.pose_covariance+=x_cov

    def update_pose_with_compass(self,heading):
        #print("updating heading {}".format(heading))
        #adjust to closest angle
        if abs(self.pose[2]-(heading-2*np.pi)) < abs(self.pose[2]-heading):
            heading=heading-2*np.pi
        if abs(self.pose[2]-(heading+2*np.pi)) < abs(self.pose[2]-heading):
            heading=heading+2*np.pi
        compass_error=0.01 #1 cm error
        measurement=np.array([0,0,heading])
        measurement_covariance=np.zeros([3,3])
        measurement_covariance[0][0]=self.sigma_too_big
        measurement_covariance[1][1]=self.sigma_too_big
        measurement_covariance[2][2]=compass_error*compass_error
        self.update_pose_with_measurement(measurement,measurement_covariance)
        #wrap back if necessary
        if self.pose[2]>np.pi:
            self.pose[2]-=2*np.pi
        if self.pose[2]<-np.pi:
            self.pose[2]+=2*np.pi
        #print("heading belief {}".format(self.pose[2]))

    def pose_to_cell(self):
        x=int(self.side_size/2+self.pose[0]/self.square_edge_length)
        y=int(self.side_size/2+self.pose[1]/self.square_edge_length)
        return [x,y]


    def get_adjacent_cells(self,cell,range=1):
        ret=[]
        for i in np.arange(-range,range+1):
            for j in np.arange(-range,range+1):
                if (i != 0) or (j != 0):
                    if (cell[0]+i)>=0 and (cell[0]+i)<self.side_size and (cell[1]+i)>=0 and (cell[1]+i)<self.side_size:
                        ret.append( [cell[0]+i,cell[1]+j] )
        return ret

    def get_cell_center(self,cell):
        return np.array([(cell[0]-self.side_size/2)*self.square_edge_length,(cell[1]-self.side_size/2)*self.square_edge_length])

    def get_angle_to_cell_center(self,start_cell,cell):
        vec=self.get_cell_center(cell)-self.get_cell_center(start_cell)
        #print("vec is {}".format(vec))
        pose_vec=np.array([ np.cos(-self.pose[2]),np.sin(-self.pose[2])])
        length=np.sqrt(vec.dot(vec))
        #print("dot product {}".format(np.dot(vec,pose_vec)/length))
        angle=np.arccos(np.dot(vec,pose_vec)/length)
        return angle


    def update_map_with_ultrasonic(self,distance,distance_uncertainty):
        #TODO this will assume I know the pose, but I should come back and use this
        #to also update the pose belief
        max_distance=3.0 #don't believe echos further than this away
        see_wall=True
        if distance>max_distance:
            see_wall=False
            distance=max_distance

        #find my cell
        start_cell=self.pose_to_cell()
        #print("start cell is {}".format(start_cell))
        closed_cells=[start_cell]
        test_cells=self.get_adjacent_cells(start_cell,3)
        while len(test_cells)!=0:
            on_cell=test_cells.pop()
            #print("on cell {}".format(on_cell))
            closed_cells.append(on_cell)
            if on_cell[0]<0 or on_cell[1]>=self.side_size or on_cell[0]<0 or on_cell[1]>=self.side_size:
                continue
            #check if this is within my cone
            angle=self.get_angle_to_cell_center(start_cell,on_cell)
            if angle<( 7*2*np.pi/360):
                vec=self.get_cell_center(on_cell)-self.get_cell_center(start_cell)
                #if in range
                celldist=np.sqrt(vec.dot(vec))
                if celldist<distance:
                    #cell is closer than what I think is blocked
                    self.map_marker_cleared[on_cell[0],on_cell[1]]+=1
                    adjacent_cells=self.get_adjacent_cells(on_cell)
                    for c in adjacent_cells:
                        if (c not in test_cells) and (c not in closed_cells):
                            test_cells.append(c)
                else:
                    #this cell is where my distance sensor says is blocked
                    if see_wall:
                        self.map_marker_blocked[on_cell[0],on_cell[1]]+=1


    def map_image(self):
        # Create a black image
        szpx = 512
        img = np.zeros((szpx,szpx,3), np.uint8)

        for i in range(self.side_size):
            for j in range(self.side_size):
                x=i*szpx/self.side_size
                y=j*szpx/self.side_size
                w=0.5*szpx/self.side_size
                color=(0,0,255)
                if self.map_marker_cleared[i,j]>self.map_marker_blocked[i,j]:
                    color=(0,0,0)
                if self.map_marker_cleared[i,j]<self.map_marker_blocked[i,j]:
                    color=(255,0,0)
                cv.rectangle(img,(int(x-w),int(y-w)),(int(x+w),int(y+w)),color,cv.FILLED)
        # Draw a diagonal blue line with thickness of 5 px
        #cv.line(img,(0,0),(511,511),(255,0,0),5)

        # Draw an arrow for my heading
        dir_unit_x=np.cos(-self.pose[2])
        dir_unit_y=np.sin(-self.pose[2])
        centerx=szpx/2
        centery=szpx/2
        arrowlen=10
        cv.arrowedLine(img,(int(centerx-arrowlen*dir_unit_x),int(centery-arrowlen*dir_unit_y)),(int(centerx+arrowlen*dir_unit_x),int(centery+arrowlen*dir_unit_y)),(255,255,255),2,8,0,0.2)

        return img
