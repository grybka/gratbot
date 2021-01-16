import numpy as np
#
import cv2 as cv

class GratbotMap():
    def __init__(self):
        self.square_edge_length=0.1 #in meters
        self.side_size=100
        self.passable_map=np.zeros([self.side_size,self.side_size])
        #0,0, is in the center

        self.kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        self.pose_angle=0
        self.pose_x=0
        self.pose_y=0
        self.pose_covariance=np.identity(3) #x,y,angle
        self.last_time=0

        self.heading_offset=0

    def update_with_compass(self,heading):
        #TODO this should be a kalman filter
        compass_error=0.01 #1 cm error
        self.pose_angle=heading

    def update_with_turn(self,turn_angle,turn_angle_unc):
        meas_cov=np.zeros([3,3])
        meas_cov[2][2]=turn_angle_unc**2
        self.pose_angle+=turn_angle
        self.pose_covariance+=meas_cov


    def map_image(self):
        # Create a black image
        szpx = 512
        img = np.zeros((szpx,szpx,3), np.uint8)
        # Draw an arrow for my heading
        dir_unit_x=np.sin(self.pose_angle)
        dir_unit_y=np.cos(self.pose_angle)
        centerx=szpx/2
        centery=szpx/2
        arrowlen=10

        cv.arrowedLine(img,(int(centerx-arrowlen*dir_unit_x),int(centery-arrowlen*dir_unit_y)),(int(centerx+arrowlen*dir_unit_x),int(centery+arrowlen*dir_unit_y)),(255,255,255),2,8,0,0.2)
        # Draw a diagonal blue line with thickness of 5 px
        #cv.line(img,(0,0),(511,511),(255,0,0),5)
        return img
