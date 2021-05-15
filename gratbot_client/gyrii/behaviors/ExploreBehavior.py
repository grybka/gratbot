from Behavior import *
from Behavior import GratbotBehaviorStatus
from MotionBehaviors import *
import time
import numpy as np
from math import sin,cos
from underpinnings.BayesianArray import BayesianArray
from gyrii.underpinnings.GratbotLogger import gprint,gprint_low
from networkx import triangular_lattice_graph,bfs_edges,subgraph_view,astar_path,connected_components

#behaviors focused on exploring the map

# From local map, find closes node with an unexplored edge
# Make that my target to go there


#TODO add path to display so I can figure out what went wrong

class ExploreBehavior(GratbotBehavior):
    def __init__(self,gridmap=None):
        self.sub_behavior=None
        self.target_node=None

    def find_good_vantage_point(self,latest_pose,shared_objects):
        with shared_objects.locks["occupancy_map"]:
            occupancy_map=shared_objects.objects["occupancy_map"]
            res=occupancy_map.resolution
            pos_grid_res=0.5
            nbins=200
            #lines=[]
            good_points=[]
            for x in np.arange(-res*nbins/2,res*nbins/2,pos_grid_res):
                for y in np.arange(-res*nbins/2,res*nbins/2,pos_grid_res):
                    position=np.array([x,y])
                    start_cell=occupancy_map.coord_to_cell(position)
                    if occupancy_map.gridmap_logodds[start_cell]>-2:
                        continue
                    uncount=0
                    max_dist=3.0
                    for angle in np.linspace(0,2*np.pi,30):
                        dist,stat=occupancy_map.raytrace_through_free(position,angle,max_dist=max_dist,breakthrough=3)
                        end_position=position+dist*np.array([np.sin(angle),np.cos(angle)])
                        if stat=="unknown":
                            uncount+=1
                    if uncount>5 and uncount < 15:
                        good_points.append(np.array([x,y]))
        if len(good_points)==0:
            gprint_low("No nodes with good vantage points")
            return None
        #get the graph map, update it if needed
        with shared_objects.locks["occupancy_map"]:
            with shared_objects.locks["graph_map"]:
                gridmap=shared_objects.objects["occupancy_map"]
                graphmap=shared_objects.objects["graph_map"]
                graphmap.describe_occupancy_map(gridmap)
        #look for close node
        best_target=None
        with shared_objects.locks["graph_map"]:
            starting_node=graphmap.get_node_nearest_to(latest_pose.vals[0:2])
            #problemhappining:  starting node is not considered passable
            target_nodes=set()
            for pt in good_points:
                target_nodes.add(graphmap.get_node_nearest_to(pt))
            best_target=graphmap.find_nearest_node_from_node_set(starting_node,target_nodes)
            path_nodes=graphmap.astar_path(starting_node,best_target)
        graphmap.focus_node=best_target
        graphmap.focus_path=path_nodes
        return best_target

    #depricated!
    def find_closest_unexplored_node(self, latest_pose,shared_objects):
        gprint("function start")
        with shared_objects.locks["graph_map"]:
            gprint("lock acquired ")
            graphmap=shared_objects.objects["graph_map"]
            #figure out which node I'm closest to
            starting_node=graphmap.get_node_nearest_to(latest_pose.vals[0:2])
            gprint("starting node is {}".format(starting_node))

            passable_view=subgraph_view(graphmap.graph,filter_edge=lambda u,v: graphmap.graph.edges[(u,v)]['passable']!='blocked',filter_node=lambda u: graphmap.graph.nodes[u]['passable']=='unknown')
            comps=connected_components(passable_view)
            unknown_node=None
            path_nodes=None
            for c in comps:
                unknown_node=graphmap.find_nearest_node_from_node_set(starting_node,c)
                #gprint("starting node {}".format(starting_node))
                try:
                    path_nodes=graphmap.astar_path(starting_node,unknown_node)
                except:
                    gprint("no path to node {}, trying next".format(unknown_node))
            if unknown_node==None:
                gprint("Warning, no node to target")
            graphmap.focus_node=unknown_node
            graphmap.focus_path=path_nodes
            #gprint("target node {}".format(target_node))
            return unknown_node


    def act(self,**kwargs):
        #gprint("explore started")
        broker=kwargs["broker"]
        shared_objects=kwargs["shared_objects"]
        if "latest_pose" not in kwargs["short_term_memory"]:
                gprint("no pose in memory aborting ")
                #in case its loading
                return GratbotBehaviorStatus.INPROGRESS
        latest_pose=BayesianArray.from_object(kwargs["short_term_memory"]["latest_pose"]["latest_pose"])
        #if we have a sub behavior, do that
        if self.sub_behavior is not None:
            step_status=self.sub_behavior.act(**kwargs)
        else:
            step_status=GratbotBehaviorStatus.COMPLETED
        if step_status==GratbotBehaviorStatus.COMPLETED:
            if self.target_node==None:
                #if the sub behavior is finished, then figure out what to do next
                gprint("Finding target node")
                #self.target_node=self.find_closest_unexplored_node(latest_pose,shared_objects)
                self.target_node=self.find_good_vantage_point(latest_pose,shared_objects)
                gprint("target node is {}".format(self.target_node))
                gprint("Waiting for confirmation")
                step_status=GratbotBehaviorStatus.INPROGRESS
            elif "script" in kwargs["short_term_memory"] and kwargs["short_term_memory"]["script"]["script"]=="go":
                gprint("GO GO GO")
                self.sub_behavior=GratbotBehavior_Series([NavigateToNode(self.target_node),GratbotBehavior_Wait(1.0)])
                step_status=GratbotBehaviorStatus.INPROGRESS
            else:
                #gprint("waiting: {}".format("script" in kwargs["short_term_memory"] ))
                step_status=GratbotBehaviorStatus.INPROGRESS
        return step_status

    def do_thing(self,pose):
        #enumerate movement options
        #let's hard code those for now
        #each of these implies a lidar and ultrasonic scan afterward
        movement_options=[['turn',15*2*np.pi/360],
                          ['turn',45*2*np.pi/360],
                          ['turn',-15*2*np.pi/360],
                          ['turn',-45*2*np.pi/360],
                          ['ahead',0.2],
                          ['ahead',-0.2]]


        #for each movement option, predict amount of new information it would goin
        info_predictions=[]
        for i in range(len(movement_options)):
            m=movement_options[i]
            if m[0]=='turn':
                #predict new pose
                p=pose.copy()
                p.vals+=np.array([0,0,m[1]])
                info_predictions.append(self.predict_info(p))
            if m[0]=='ahead':
                #predict new pose
                p=pose.copy()
                p.vals+=np.array([m[1]*sin(p.vals[2]),m[1]*cos(p.vals[2]),0])
                info_predictions.append(self.predict_info(p))
            print("A {} of {} should give info of {}".format(m[0],m[1],info_predictions[i]))
        best_i=np.argmax(info_predictions)
        return movement_options[best_i],info_predictions[best_i]
