#A graph that represents a 2d map for robot navigation

import numpy as np
from networkx import triangular_lattice_graph,bfs_edges,subgraph_view,astar_path
from gyrii.underpinnings.GridGeometry import bresenham
import cv2 as cv


class GraphMap:
    def __init__(self,total_length,length_scale):
        self.total_length=total_length #total length the graph should span in meters
        self.length_scale=length_scale #length between nodes in meters
        #self.lattice_size=int(2*total_length/(np.sqrt(3)*length_scale)) #number of rows and columns in grid
        self.center_offset=np.array([-total_length/2,-total_length/2])
        #build the graph
        nrow=int(2*total_length/(np.sqrt(3)*length_scale)) #number of rows and columns in grid
        ncol=int(2*total_length/length_scale)
        self.graph=triangular_lattice_graph(nrow,ncol)
        #update so the "pos" property is a numpy array in position space
        for node in self.graph.nodes:
            x=self.graph.nodes[node]['pos']
            newx=x[0]*self.length_scale+self.center_offset[0]
            newy=x[1]*self.length_scale+self.center_offset[1]
            self.graph.nodes[node]['pos']=np.array([newx,newy])
            self.graph.nodes[node]['passable']='unknown'
        for edge in self.graph.edges:
            self.graph.edges[edge]['passable']='unknown'

        #this is used mostly for drawing
        self.focus_node=None
        self.focus_path=None

    def describe_occupancy_map(self,occmap,free_cutoff=-2,blocked_cutoff=2,search_range=2):
        #occmap is a occupancymap2d
        #searchrang is the number of pixels to look for blocked or unknown cells
        for node in self.graph.nodes:
            pos=self.graph.nodes[node]['pos']
            coord=occmap.coord_to_cell(pos)
            largest=np.max(occmap.gridmap_logodds[max(coord[0]-search_range,0):coord[0]+search_range,max(coord[1]-search_range,0):coord[1]+search_range])
            if largest<free_cutoff:
                self.graph.nodes[node]['passable']='free'
            elif largest<blocked_cutoff:
                self.graph.nodes[node]['passable']='unknown'
            else:
                self.graph.nodes[node]['passable']='blocked'
        #identify free edges, blocked edges, and unknown edges
        for edge in self.graph.edges:
            #if mygraph.nodes[edge[0]]['free'] or mygraph.nodes[edge[1]]['free']:
            p1=occmap.coord_to_cell(self.graph.nodes[edge[0]]['pos'])
            p2=occmap.coord_to_cell(self.graph.nodes[edge[1]]['pos'])
            allpts=bresenham(p1,p2)
            largest=np.max( occmap.gridmap_logodds[allpts[:,0],allpts[:,1]]) #figuring out this indexing took some work
            if largest<free_cutoff:
                self.graph.edges[edge]['passable']='free'
            elif largest<blocked_cutoff:
                self.graph.edges[edge]['passable']='unknown'
            else:
                self.graph.edges[edge]['passable']='blocked'

    def get_point_locations(self,passable):
        ret=[]
        for node in self.graph.nodes:
            if self.graph.nodes[node]['passable']==passable:
                ret.append(self.graph.nodes[node]['pos'])
        return np.array(ret)

    def get_edge_locations(self,passable):
        ret=[]
        for edge in self.graph.edges:
            if self.graph.edges[edge]['passable']==passable:
                ret.append([self.graph.nodes[edge[0]]['pos'],self.graph.nodes[edge[1]]['pos']])
        return np.array(ret)

    def get_node_nearest_to(self,position):
        scaled_position=(position-self.center_offset[0])/self.length_scale
        #xx = (0.5 * (j % 2) + i for i in cols for j in rows)
        i=int(scaled_position[0])
        h = np.sqrt(3) / 2
        #yy = (h * j for i in cols for j in rows)
        j=int(scaled_position[1]/h)
        closest_node=( i, j)
        closest_dist=np.linalg.norm(self.graph.nodes[ (i,j) ]['pos']-position)
        #I guess I could keep track of int rounding and half steps but I don't think this search takes long
        for node in self.graph.neighbors( (i,j) ):
            dist=np.linalg.norm(self.graph.nodes[node]['pos']-position)
            if dist<closest_dist:
                closest_dist=dist
                closest_node=node

        return closest_node

    def find_nearest_edge(self,start_node,passable):
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']!='blocked')
        for edge in bfs_edges(passable_view,start_node):
            if self.graph.edges[edge]['passable']==passable:
                return edge
        return None

    def count_accessible_nodes(self,start_node,passable_status,max_count=10):
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']!='blocked',filter_node=lambda u: self.graph.nodes[u]['passable']==passable_status)
        count=0
        for edge in bfs_edges(passable_view,start_node):
            max_count+=1
        return count



    def find_nearest_passable_node(self,start_node,passable):
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']!='blocked',filter_node=lambda u: self.graph.nodes[u]['passable']!='blocked')
        for edge in bfs_edges(passable_view,start_node):
            if self.graph.nodes[edge[0]]['passable']==passable:
                print(self.graph.nodes[edge[0]])
                return edge[0]
            if self.graph.nodes[edge[1]]['passable']==passable:
                print(self.graph.nodes[edge[1]])
                return edge[1]
        return None


    def find_nearest_passable_node_adjacent_to_set(self,start_node,node_set):
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']!='blocked',filter_node=lambda u: self.graph.nodes[u]['passable']!='blocked')
        for edge in bfs_edges(passable_view,start_node):
            if self.graph.nodes[edge[0]]['passable']=='free' and edge[1] in node_set:
                return edge[0]
            if self.graph.nodes[edge[1]]['passable']=='free' and edge[0] in node_set:
                return edge[1]
        return None

    def find_nearest_node_from_node_set(self,start_node,node_set):
        #passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']!='blocked',filter_node=lambda u: self.graph.nodes[u]['passable']!='blocked')
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']=='free',filter_node=lambda u: self.graph.nodes[u]['passable']=='free')
        for edge in bfs_edges(passable_view,start_node):
            if edge[0] in node_set:
                return edge[0]
            if edge[1] in node_set:
                return edge[1]
        return None

    def astar_path(self,start_node,end_node):
        passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']=='free',filter_node=lambda u: self.graph.nodes[u]['passable']=='free')
        #passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']=='free')
        def dist(a, b):
            (x1, y1) = a
            (x2, y2) = b
            return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        shortest_path=astar_path(passable_view,start_node,end_node,heuristic=dist) #I can also add path weights if that's a thing
        return shortest_path

    def opencv_apply_to_image(self,image,start_x,start_y,stop_x,stop_y):
        #note y is flipped heer
        #passable_view=subgraph_view(self.graph,filter_edge=lambda u,v: self.graph.edges[(u,v)]['passable']=='free')
        passable_view=subgraph_view(self.graph,filter_node=lambda u: self.graph.nodes[u]['passable']=='free')
        def convert_to_image(pos,start_x,start_y,stop_x,stop_y):
            newx=int((pos[0]-start_x)*image.shape[0]/(stop_x-start_x))
            newy=image.shape[1]-int((pos[1]-start_y)*image.shape[1]/(stop_y-start_y))
            return (newx,newy)
        for node in passable_view:
            centr=convert_to_image(passable_view.nodes[node]['pos'],start_x,start_y,stop_x,stop_y)
            #cv.circle(image,centr,3, (255,0,0), 2)
            cv.circle(image,centr,1, (255,0,0), 1)
        if self.focus_node is not None:
            centr=convert_to_image(self.graph.nodes[self.focus_node]['pos'],start_x,start_y,stop_x,stop_y)
            cv.circle(image,centr,3, (0,0,255), 2)
        if self.focus_path is not None:
            prev=None
            for p in self.focus_path:
                if prev!=None:
                    start=convert_to_image(self.graph.nodes[prev]['pos'],start_x,start_y,stop_x,stop_y)
                    stop=convert_to_image(self.graph.nodes[p]['pos'],start_x,start_y,stop_x,stop_y)
                    cv.line(image,start,stop,(0,0,255),2)
                prev=p
