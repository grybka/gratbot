from vector2d import intersectLines,cross2d, angle_between
import numpy as np
import cv2
import random

def angle_to_unit_vector(angle):
    return np.array([ np.sin(angle),np.cos(angle)])

def point_to_int_tuple(x):
    return (int(x[0]),int(x[1]) )

class MouseWorldWall:
    def __init__(self,x0=np.zeros(2),x1=np.zeros(2)):
        self.x0=x0
        self.x1=x1
class MouseWorldObject:
    def __init__(self,name,x=np.zeros(2)):
        self.position=x
        self.name=name

class MouseWorld:
    def __init__(self):
        self.walls=[]
        self.objects=[]
        self.mouse_pos=np.zeros(2)
        self.mouse_facing=0 #angle respect to [0,1]
        self.mouse_speed=0.1
        self.dist_eps=0.001
        self.angle_eps=0.0001
        self.turn_radius=0.4
        self.bounds=[100,100]

        self.last_motor_speed=0
        self.last_turning=0

    #inputs are [motor_speed,servo_yaw]
    # motor_speed in is [-1,1]
    # servo yaw is in [-1,1] with 0 being straight ahead

    #observations are [dist_to_wall, cheese_target(5) ]
    #reward is ???
    def step(self,action_array):
        motor_speed=action_array[0]
        self.last_motor_speed=motor_speed
        turning_x=action_array[1]
        self.last_turning=turning_x
        if turning_x==0:
            turning_x=self.angle_eps
        speed=self.mouse_speed*motor_speed
        turn_rad=self.turn_radius/turning_x
#print("turn rad {}".format(turn_rad))
        dtheta=speed/(turn_rad)
        #figure out bot stop position
        angle_change=dtheta
        displacement=speed*angle_to_unit_vector(self.mouse_facing+0.5*dtheta)
        new_angle=self.mouse_facing+dtheta
        new_position=self.mouse_pos+displacement
        #find intersections with walls
        smallest_s=100000
        for wall in self.walls:
            #first just straight ahead
            xi, yi, valid, r, s=intersectLines( wall.x0,wall.x1,self.mouse_pos,self.mouse_pos+angle_to_unit_vector(self.mouse_facing))
            if s>0 and s<smallest_s:
                smallest_s=s
            #now intersect with path
            xi, yi, valid, r, s=intersectLines( wall.x0,wall.x1,self.mouse_pos,new_position)
            if valid:
                hit_pos=self.mouse_pos+(new_position-self.mouse_pos)*(s-self.dist_eps)
                new_angle=self.mouse_facing+dtheta*s
                new_position=hit_pos
        #update position
        self.mouse_facing=new_angle
        self.mouse_pos=new_position
        #calculate reward
        reward=0
        #TODO calculate vision
        #TODO calculate wall distanc
        dist_to_wall=smallest_s
        if dist_to_wall>0.5: #this is the maximum distance it can detect a wall, perhaps
            dist_to_wall=0.5 
        angle=self.cheese_detection()
        #step 0, don't hit walls.  negative reward for being too close to a wall
        if dist_to_wall<0.1:
            reward-=0.1
        #step 1, if you can see the cheese, some reward
        reward+=0.05*angle[0]+0.1*angle[1]+0.2*angle[2]+0.1*angle[3]+0.05*angle[4]

        observation=np.append(angle,smallest_s)
        #find markers
        marks=self.object_detection("marker")
        observation=np.append(observation,marks)
        return observation,reward
#return [ dist_to_wall ],reward
    
        

    def add_boundary(self,width,height):
        self.bounds=[width,height]
        pts=[]
        pts.append(np.array([-width/2,-height/2]))
        pts.append(np.array([-width/2,height/2]))
        pts.append(np.array([width/2,height/2]))
        pts.append(np.array([width/2,-height/2]))
        for i in range(4):
            j=(i+1)%4
            self.walls.append(MouseWorldWall(pts[i],pts[j]))

    def add_random_cheese(self):
        loc=np.array([ random.uniform(-self.bounds[0]/2,self.bounds[0]/2),random.uniform(-self.bounds[1]/2,self.bounds[1]/2) ])
        self.objects.append(MouseWorldObject("cheese",loc))

    def add_corner_markers(self,width,height):
        self.objects.append(MouseWorldObject("marker",np.array([-width/2,-height/2])))
        self.objects.append(MouseWorldObject("marker",np.array([-width/2,height/2])))
        self.objects.append(MouseWorldObject("marker",np.array([width/2,height/2])))
        self.objects.append(MouseWorldObject("marker",np.array([width/2,-height/2])))


    def angle_to_position(self,position):
#angle between mouse facing and position
        vtobj=position-self.mouse_pos
        return angle_between(vtobj,angle_to_unit_vector(self.mouse_facing))

    def render(self):
        scale=100
        offset=np.array( [320,200] )
        image=np.zeros( (480,640,3) )
        for i in range(len(self.walls)):
            x=point_to_int_tuple(scale*self.walls[i].x0+offset)
            y=point_to_int_tuple(scale*self.walls[i].x1+offset)
            cv2.line(image,x,y,(255,0,0),2)
        for i in range(len(self.objects)):
            x=point_to_int_tuple(scale*self.objects[i].position+offset)
            if self.objects[i].name=="cheese":
                cv2.circle(image,x,10,(0,255,0),2)
            else:
                cv2.circle(image,x,5,(100,100,100),2)

        mouse_size=0.1
        center=point_to_int_tuple(scale*self.mouse_pos+offset)
        tip=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing))+offset)
        lpoint=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing+np.pi*2/3))+offset)
        rpoint=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing-np.pi*2/3))+offset)
        cv2.line(image,tip,lpoint,(0,0,255),2)
        cv2.line(image,tip,rpoint,(0,0,255),2)
        cv2.line(image,center,lpoint,(0,0,255),2)
        cv2.line(image,center,rpoint,(0,0,255),2)

#print(self.mouse_speed)


        indicator_origin=np.array([320,400])
        indicator_scale=60
        indicator_tip=indicator_origin+np.array([0,indicator_scale*self.last_motor_speed])
        turn_indicator_tip=indicator_origin+np.array([indicator_scale*self.last_turning,0])

        cv2.line(image,point_to_int_tuple(indicator_origin),point_to_int_tuple(indicator_tip),(255,0,0),2)
        cv2.line(image,point_to_int_tuple(indicator_tip),point_to_int_tuple(indicator_tip-np.array([10,10])),(255,0,0),2)
        cv2.line(image,point_to_int_tuple(indicator_tip),point_to_int_tuple(indicator_tip-np.array([-10,10])),(255,0,0),2)
        cv2.line(image,point_to_int_tuple(indicator_origin),point_to_int_tuple(turn_indicator_tip),(255,0,0),2)

            
        cv2.imshow('image',image)


    def cheese_detection(self):
        return self.object_detection("cheese")

    def object_detection(self,name):
        n_detectors=5
        fov=np.pi/2
        sigma=fov/n_detectors
        detector_centers=np.linspace(-fov/2,fov/2,n_detectors)
        detections=np.zeros(n_detectors)
        for i in range(len(self.objects)):
            if self.objects[i].name!=name:
                continue
            vtobj=self.objects[i].position-self.mouse_pos
            angle=(np.arctan2(vtobj[0],vtobj[1]))-self.mouse_facing
            while angle<(-np.pi):
                angle=angle+2*np.pi
            if angle>(-fov/2) and angle<(fov/2):
                for j in range(n_detectors):
                    x=(angle-detector_centers[j])/sigma
                    excite=0.5*np.exp(-x*x/2)/(np.sqrt(2*np.pi)*sigma)
                    detections[j]+=excite
        for j in range(n_detectors):
            detections[j]=min(detections[j],1.0)
        return detections
