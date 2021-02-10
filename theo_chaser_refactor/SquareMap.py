
from BayesianArray import BayesianArray
import numpy as np
from uncertainties.umath import cos,sin


def line_intersection(pt1,pt2,x,n):
    #line is from pt 1 to pt 2
    #I'm asking if I have a point x, and a normalized direction n, at what point betwen pt1 and pt2 does it hit?
    #returns distance along each line, and point of intersection
    a=pt2-pt1
    k=x-pt1
    mat=np.array([[ a[0],-n[0]],[a[1],-n[1]]])
    sol=np.array([k[0],k[1]])
    b=np.linalg.solve(mat,sol)
    if b[0]<0 or b[0]>1:
        return None,None #solution not on line
    return b,pt1+b[0]*(pt2-pt1) #b1 is distance between x and point

def closest_point_on_line(pt1,pt2,pt3):
    p2p1n=(pt2-pt1)/np.linalg.norm(pt2-pt1)
    thept=pt1+p2p1n*(p2p1n.dot(pt3-pt1))
    return thept


def closest_point_on_line_segment(pt1,pt2,pt3):
    p2p1n=(pt2-pt1)/np.linalg.norm(pt2-pt1)
    b=(p2p1n.dot(pt3-pt1))/np.linalg.norm(pt2-pt1)
    if b>1:
        return pt2
    if b<0:
        return pt1
    return pt1+(pt2-pt1)*b

class Rectangle:
    def __init__(self,center,lxly,theta):
        self.center=center
        self.lxly=lxly
        self.theta=theta

    def get_corner(self,nth):
        #offset=nth*np.pi/2
        theta=self.theta
        lxly=self.lxly
        xdisp=lxly[0]*(((nth+1)>>1)%2-0.5) # -0.5 0.5 -0.5 0.5
        ydisp=lxly[1]*((nth>>1)%2-0.5)# -0.5 -0.5 0.5 0.5
        rot=np.array( [ [np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        return self.center+rot.dot( np.array([xdisp,ydisp]))

    def get_edges(self):
        ret=[]
        for i in range(4):
            ret.append( [ self.get_corner(i),self.get_corner(i+1)])
        return ret



def predict_ultrasonic_result(rect,pose_x,pose_theta,opening_angle):
    pose_n=np.array( [-np.cos(pose_theta),-np.sin(pose_theta)])
    edges=rect.get_edges()
    dists=[]
    for edge in edges:
        #if the closest point lies on the line within my angle, I'm done
        closest=closest_point_on_line(edge[0],edge[1],pose_x)
        angle=np.arccos(pose_n.dot(closest-pose_x)/np.linalg.norm(closest-pose_x))
        if abs(angle)<abs(opening_angle):
            print("success")
            dists.append(np.linalg.norm(closest-pose_x))
        else:
            high_pose_n=np.array( [-np.cos(pose_theta+opening_angle/2),-np.sin(pose_theta+opening_angle/2)])
            low_pose_n=np.array( [-np.cos(pose_theta-opening_angle/2),-np.sin(pose_theta-opening_angle/2)])
            b1,pt1=line_intersection(edge[0],edge[1],pose_x,high_pose_n)
            b2,pt2=line_intersection(edge[0],edge[1],pose_x,low_pose_n)
            if b1 is not None and b1[1]>0:
                dists.append(b1[1])
            if b2 is not None and b1[1]>0:
                dists.append(b2[1])
    if len(dists)==0:
        return None
    print("dists {}".format(dists))
    return np.min(dists)





class UltrasonicMeasurement:
    def __init__(self,x=0,y=0,theta=0,dist=0):
        #where we were when we took the measurement
        self.xythetadistance=BayesianArray([x,y,theta,dist],[[1e-4,0,0,0],[0,1e-4,0,0],[0,0,100,0],[0,0,0,100]])

    def to_pt(self):
        [x,y,theta,dist]=self.xythetadistance.get_as_ufloat()
        new_y=y-dist*cos(theta)
        new_x=x-dist*sin(theta)
        return [new_x,new_y]
