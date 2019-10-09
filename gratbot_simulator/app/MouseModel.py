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


    #inputs are [motor_speed,servo_yaw]
    # motor_speed in is [-1,1]
    # servo yaw is in [-1,1] with 0 being straight ahead

    #observations are [dist_to_wall, cheese_target(5) ]
    #reward is ???
    def step(self,action_array):
        motor_speed=action_array[0]
        turning_x=action_array[1]
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
        for wall in self.walls:
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
        dist_to_wall=0
        return [ dist_to_wall ],reward
    
        

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
            cv2.circle(image,x,10,(0,255,0),2)

        mouse_size=0.1
        center=point_to_int_tuple(scale*self.mouse_pos+offset)
        tip=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing))+offset)
        lpoint=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing+np.pi*2/3))+offset)
        rpoint=point_to_int_tuple(scale*(self.mouse_pos+mouse_size*angle_to_unit_vector(self.mouse_facing-np.pi*2/3))+offset)
        cv2.line(image,tip,lpoint,(0,0,255),2)
        cv2.line(image,tip,rpoint,(0,0,255),2)
        cv2.line(image,center,lpoint,(0,0,255),2)
        cv2.line(image,center,rpoint,(0,0,255),2)
            
        cv2.imshow('image',image)


