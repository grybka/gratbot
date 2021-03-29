#some integer geometry functions for drawing on grids
import math
import numpy as np

def triangle_inside(A,B,C):
    #on an integer grid, return all points inside the triangle formed by A,B,C
    ret=[]
    #Order them from least to greatest Y
    if B[1]<A[1]:
        A,B=B,A
    if C[1]<A[1]:
        C,A=A,C
    if C[1]<B[1]:
        C,B=B,C

    if A[1]==C[1]: #trivial case of straight line
        for x in range(int(math.floor(min(A[0],B[0],C[0]))),int(math.ceil(max(A[0],B[0],C[0])))):
            ret.append( (x,A[0]) )
            return ret
    islope_ac=(C[0]-A[0])/(C[1]-A[1])
    if A[1]!=B[1]:#trivial case of no lower triangle
        islope_ab=(B[0]-A[0])/(B[1]-A[1])
        for y in range(int(math.floor(A[1])),int(math.ceil(B[1]))):
            leftside=islope_ac*(y-A[1])+A[0]
            rightside=islope_ab*(y-A[1])+A[0]
            if rightside<leftside:
                leftside,rightside=rightside,leftside
            leftside=int(math.floor(leftside))
            rightside=int(math.ceil(rightside))
            for x in range(leftside,rightside):
                ret.append( (x,y) )
    if C[1]!=B[1]: #trivial case of no upper triangle
        islope_bc=(C[0]-B[0])/(C[1]-B[1])
        for y in range(int(math.floor(B[1])),int(math.ceil(C[1]))):
            leftside=islope_ac*(y-A[1])+A[0]
            rightside=islope_bc*(y-B[1])+B[0]
            if rightside<leftside:
                leftside,rightside=rightside,leftside
            leftside=int(math.floor(leftside))
            rightside=int(math.ceil(rightside))
            for x in range(leftside,rightside):
                ret.append( (x,y) )
    return ret

def bresenham(start, end, returnset=False):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    if returnset:
        ret=set()
        for i in range(len(points)):
            ret.add((points[i][0],points[i][1]))
        return ret
    points = np.array(points)
    return points
