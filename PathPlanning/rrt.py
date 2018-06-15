#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 15 07:13:40 2018

@author: kartikmadhira
"""

#creating a rrtgraph object.
import matplotlib.pyplot as plt
import math
import random



show_animation=True

class Node(object):
    def __init__(self,x_cor,y_cor):
        self.x_cor=x_cor
        self.y_cor=y_cor
        self.edges=[]

class Edge(object):
    def __init__(self,start_node,end_node,node_val):
        self.start_node=start_node
        self.end_node=end_node
        self.node_val=node_val

#the constructor is a list of nodes and edges
class RRT(object):
    def __init__(self,edges=[],nodes=[]):
        self.root=None
        self.edges=edges
        self.nodes=nodes
        
    def set_root(self,start_x,start_y):
        a=Node(start_x,start_y)
        self.nodes.append(a)
        self.root=a
        
    def add_edge(self,from_x,from_y,to_x,to_y,distance_val):
        from_node=None
        to_node=None
        #check for any nodes present, if yes then take only that node to
        #disallow duplication
        
        for nodes in self.nodes:
            if((from_x,from_y)==(nodes.x_cor,nodes.y_cor)):
                from_node=nodes
                print("lol")
            if((to_x,to_y)==(nodes.x_cor,nodes.y_cor)):
                to_node=nodes
        #but if from node or to node is empty then create those nodes
       
        if(from_node==None):
            from_node=Node(from_x,from_y)
            self.nodes.append(from_node)
        if(to_node==None):
            to_node=Node(to_x,to_y)
            self.nodes.append(to_node)
         
        #finally creating an edge and adding edge to edges[] list
        new_edge=Edge(from_node,to_node,distance_val)
        self.edges.append(new_edge)
        
        #also adding the edge to respective nodes's lists
        from_node.edges.append(new_edge)
        to_node.edges.append(new_edge)
        
        return from_node,to_node
    
    def draw_tree(self):
        plt.clf()
        for edges in self.edges:
            from_node_x=edges.start_node.x_cor
            from_node_y=edges.start_node.y_cor
            to_node_x=edges.end_node.x_cor
            to_node_y=edges.end_node.y_cor
            plt.plot([from_node_x,to_node_x],[from_node_y,to_node_y],"-r")
            plt.pause(0.0001)
        plt.axis([0, 60, 0, 60])
        plt.grid(True)
        plt.pause(1)
        plt.show()
      
    def find_nearest_node(self,from_x,from_y):
        smallest=(10000,None)
        
        for nodes in self.nodes:
            d=math.sqrt((from_x-nodes.x_cor)**2+(from_y-nodes.y_cor)**2)
            #print(nodes)
            if(d<smallest[0]):
                smallest=(d,nodes)
        print(smallest[1].x_cor,smallest[1].y_cor)
        self.add_edge(smallest[1].x_cor,smallest[1].y_cor,from_x,from_y,2)
    
def main():
    ox=[]
    oy=[]
    plt.clf()
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    
    if show_animation:
        plt.plot(ox, oy, ".k")
     
    rrt=RRT()
    rrt.set_root(10,10)
    plt.plot(10, 10, "xr")
    
    for i in range(100):
        from_x=random.randint(0,60)
        from_y=random.randint(0,60)
        rrt.find_nearest_node(from_x,from_y)
    rrt.draw_tree()
    
    
    
    
    
    
    """
    rrt.add_edge(10,10,15,30,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(15,30,50,40,math.sqrt((10-40)**2+(20-30)**2))
    rrt.add_edge(10,10,15,30,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(50,40,44,33,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(15,30,5,4,math.sqrt((10-20)**2+(20-30)**2))
    rrt.draw_tree()"""
    
if __name__ == '__main__':
    main()   
    
    