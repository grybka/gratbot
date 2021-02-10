import numpy as np
#
import cv2 as cv
from BayesianArray import BayesianArray
import uuid
from uncertainties import ufloat
from uncertainties.umath import *

from id_to_name import id_to_name
from bresenham import bresenhamline

class GratbotMapStaticObject():
    def __init__(self,label):
        self.id=uuid.uuid1()
        self.label=label
        self.position=BayesianArray([0,0],[[100,0],[0,100]])

    def __str__(self):
        return "Static Object id {} at {} with label {}".format(self.id,self.position,self.label)

class GratbotMap():
    def __init__(self):
        self.square_edge_length=0.1 #in meters
        self.side_size=100
        self.map_marker_cleared=np.zeros([self.side_size,self.side_size])
        self.map_marker_blocked=np.zeros([self.side_size,self.side_size])
        #0,0, is in the center

        #self.kfx = KalmanFilter(dim_x=2,dim_z=1) #xxpos, xvel,      xmeas
        #self.pose=np.zeros(3) #x y angle
        #self.pose_covariance=1e-4*np.identity(3) #x,y,angle
        #self.pose_covariance[2][2]=np.pi*np.pi # idon't know starting angle
        self.pose=BayesianArray([0,0,0],[[1e-4,0,0],[0,1e-4,0],[0,0,100]])
        self.pose_min_covariance=1e-4*np.array([1,1,1])
        self.last_time=0
        self.heading_offset=0
        self.sigma_too_big=1e10

        self.tracked_objects={}
        self.min_tracked_object_covariance=np.array([1e-4,1e-4])

    def get_heading(self):
        return self.pose.vals[2],np.sqrt(self.pose.covariance[2][2])
        #return self.pose[2],np.sqrt(self.pose_covariance[2][2])

#    def update_pose_with_measurement(self,x_vec,x_cov): #x,y,angle and covariance
#        #use this if I made a measurement of the pose, not a change
#        mu_1=x_vec
#        mu_2=self.pose
#        sigma_1=x_cov
#        sigma_2=self.pose_covariance
#        sumsigmainv=np.linalg.inv(sigma_1+sigma_2)
#        mu_new=np.dot(sigma_2,np.dot(sumsigmainv,mu_1))+np.dot(sigma_1,np.dot(sumsigmainv,mu_2))
#        #print("mu new {}".format(mu_new))
#        sigma_new=np.dot(sigma_1,np.dot(sumsigmainv,sigma_2))+self.pose_min_covariance
#        self.pose=mu_new
#        self.pose_covariance=sigma_new

    def angle_dist_to_map_space(self,angle,dangle,dist,ddist):
        (myx,myy,myangle)=self.pose.get_as_ufloat()
        uangle=-ufloat(angle,dangle)+myangle
        udist=ufloat(dist,ddist)
        itsx=myx-udist*sin(uangle) #remember north is -y
        itsy=myy-udist*cos(uangle)
        spot=BayesianArray.from_ufloats([itsx,itsy])
        return spot

    def update_object_with_vid(self,id,spot,label):
        #print("updating track {} with {}".format(self.tracked_objects[id],spot))
        #print("updating a {} with {}".format(label,spot))
        self.tracked_objects[id].position=self.tracked_objects[id].position.updated(spot).min_covariance(self.min_tracked_object_covariance)
        #print("now {} ".format(self.tracked_objects[id]))

    def associate_map_object_with_vid(self,angle,dangle,dist,ddist,label):
        spot=self.angle_dist_to_map_space(angle,dangle,dist,ddist)
        for key in self.tracked_objects:
            obj=self.tracked_objects[key]
            dx=obj.position.get_as_ufloat()-spot.get_as_ufloat()
            dist_squared=dx.dot(dx)
            dsigma=dist_squared.n/dist_squared.std_dev
            print("trying to associate with {}, dist {}".format(obj.id,dsigma))
            #let's try, if its within 1 sigma with correct label, then associate it
            if dsigma<1:
                return obj.id
        return None

    def vid_map_merit(self,obj,trackarray):
        mislabel_penalty=-0.5
        dx=obj.position.get_as_ufloat()-trackarray[1].get_as_ufloat()
        #print("{} {} ".format(obj.position.get_as_ufloat(),trackarray[1].get_as_ufloat()))
        dist_squared=dx.dot(dx)
        dsigma=max(dist_squared.n/dist_squared.std_dev,0.5)
        merit=-dsigma
        if trackarray[0]!=obj.label:
            merit+=mislabel_penalty
        return merit #TODO fix

    def update_with_vid_objects(self,tracker):
        tracks_to_assign=[]
        min_ok_merit=-2.0
        min_seen_frames=5
        for obj in tracker.tracked_objects:
            if obj.seen_frames>min_seen_frames:
                angle,dangle=tracker.predict_angle_from_track(obj)
                dist,ddist=tracker.predict_dist_from_track(obj)
                spot=self.angle_dist_to_map_space(angle,dangle,dist,ddist)
                tracks_to_assign.append([ obj.label,spot,obj.id]) #TODO include confidence
        for key in self.tracked_objects: #TODO if I start storing many objects in memory, I should iterate over detections instead
            if len(tracks_to_assign)==0:
                continue
            obj=self.tracked_objects[key]
            merit_array=[]
            for i in range(len(tracks_to_assign)):
                merit_array.append(self.vid_map_merit(obj,tracks_to_assign[i]))
            best_fit=np.argmax(merit_array)
            if merit_array[best_fit]>min_ok_merit:
                self.update_object_with_vid(key,tracks_to_assign[best_fit][1],tracks_to_assign[best_fit][0])
                tracker.retrieve_object(tracks_to_assign[best_fit][2]).associated_map_object=key
                tracks_to_assign.remove(tracks_to_assign[best_fit])
            else:
                pass #do I do anything if I can't associate it?  No I suppose not
        if len(tracks_to_assign)>0:
            for i in range(len(tracks_to_assign)):
                new_obj=GratbotMapStaticObject(tracks_to_assign[i][0])
                self.tracked_objects[new_obj.id]=new_obj
                self.update_object_with_vid(new_obj.id,tracks_to_assign[i][1],tracks_to_assign[i][0])


    def change_pose_with_offset(self,x_vec,x_cov):
        #use this if I made a -change- to the pose, not a measurement
        self.pose=self.pose+BayesianArray(x_vec,x_cov)

    def update_pose_with_compass(self,heading):
        #print("updating heading {}".format(heading))
        #adjust to closest angle
        if abs(self.pose.vals[2]-(heading-2*np.pi)) < abs(self.pose.vals[2]-heading):
            heading=heading-2*np.pi
        if abs(self.pose.vals[2]-(heading+2*np.pi)) < abs(self.pose.vals[2]-heading):
            heading=heading+2*np.pi
        compass_error=0.20 #10 degree error, sadly
        spot=BayesianArray([0,0,heading],[[self.sigma_too_big,0,0],[0,self.sigma_too_big,0],[0,0,compass_error*compass_error]])
        self.pose=self.pose.updated(spot).min_covariance(self.pose_min_covariance)
        #TODO min covariance
        #wrap back if necessary
        if self.pose.vals[2]>np.pi:
            self.pose.vals[2]-=2*np.pi
        if self.pose.vals[2]<-np.pi:
            self.pose.vals[2]+=2*np.pi
        #print("heading belief {}".format(self.pose[2]))

    def pose_to_cell(self):
        x=int(self.side_size/2+self.pose.vals[0]/self.square_edge_length)
        y=int(self.side_size/2+self.pose.vals[1]/self.square_edge_length)
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
        pose_vec=np.array(self.get_pose_unit_vector())
        length=np.sqrt(vec.dot(vec))
        #print("dot product {}".format(np.dot(vec,pose_vec)/length))
        angle=np.arccos(np.dot(vec,pose_vec)/length)
        return angle

    def get_avg_expected_ultrasonic(self):
        angles=np.linspace(-7.5,7.5,10)
        for angle in angles:
            ang=self.pose.vals[3]+angle*2*np.pi/360
            pointing=np.array( -np.sin(ang),-np.cos(ang))
            start=np.array(self.pose_to_cell(self))
            end=start+20*pointing
            end=np.array([int(end[0]),int(end[1])])
            points=bresenhamline(start,end)
            #walk along the pointing vector until I either hit something or not
            dist_sum=0
            for i in range(len(points)):
                if points[0]<0 or points[1]<0 or points[0]>self.side_size or points[1]>self.side_size or self.map_marker_blocked[points[i][0]][points[i][1]]>self.map_marker_blocked[points[i][0]][points[i][1]]:


                    vec=self.get_cell_center(points[i])-self.get_cell_center(start)
                    celldist=np.sqrt(vec.dot(vec))
                    dist_sum+=celldist

            return dist_sum/len(npoints)


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
            if abs(angle)<( 7*2*np.pi/360):
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

    def get_pose_unit_vector(self):
        dir_unit_y=-np.cos(self.pose.vals[2])
        dir_unit_x=-np.sin(self.pose.vals[2])
        return [dir_unit_x,dir_unit_y]

    def get_pose_unit_vector_as_ufloat(self):
        mypose=self.pose.get_as_ufloat()
        dir_unit_y=-cos(mypose[2])
        dir_unit_x=-sin(mypose[2])
        return [dir_unit_x,dir_unit_y]

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

        # Draw an arrow for my heading  #An angle of zero degrees should be in the -y direction
        # An angle of 90 degrees should be in the
        [dir_unit_x,dir_unit_y]=self.get_pose_unit_vector()
        centerx=szpx/2
        centery=szpx/2
        arrowlen=10
        cv.arrowedLine(img,(int(centerx-arrowlen*dir_unit_x),int(centery-arrowlen*dir_unit_y)),(int(centerx+arrowlen*dir_unit_x),int(centery+arrowlen*dir_unit_y)),(255,255,255),2,8,0,0.2)

        #pxels per meter
        self.scale=szpx/(self.square_edge_length*self.side_size)
        #Draw my tag_objects
        for obj in self.tracked_objects:
            x,y,h1,h2,theta=self.tracked_objects[obj].position.get_error_ellipse()
            #I'me very confused about my axis flipping
            ex=int(self.scale*x+centerx)
            ey=int(self.scale*y+centery)
            #print("ex {} ey {}".format(ex,ey))

            color = (150, 150, 150)
            thickness = 5
            cv.ellipse(img,(ex,ey),( int(h1*self.scale),int(h2*self.scale)),-theta*180/np.pi,0,360,color,thickness)
            font = cv.FONT_HERSHEY_SIMPLEX
            # fontScale
            fontScale = 0.4
            # Red color in BGR
            color = (255, 255, 255)
            # Line thickness of 2 px
            thickness = 1
            text=id_to_name(obj)
            org=( ex,ey)
            image = cv.putText(img, text, org, font,  fontScale, color, thickness, cv.LINE_AA)
        return img
