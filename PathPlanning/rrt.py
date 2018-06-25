"""
Basic implementation of Rapidly Exploring Random Trees(RRT)
References:
1) http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf
2) Simulation plots from: 
   https://github.com/AtsushiSakai/PythonRobotics
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
    def __init__(self,start_node,end_node,node_val,path=dict()):
        self.start_node=start_node
        self.end_node=end_node
        self.node_val=node_val
        

#the constructor is a list of nodes and edges
class RRT(object):
    def __init__(self,edges=[],nodes=[],path=dict(),step_dist=3,cost_radius=4):
        self.root=None
        self.edges=edges
        self.nodes=nodes
        self.path=path
        self.step_dist=step_dist
        self.cost_radius=cost_radius
        
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
    
    def draw_tree(self,obstacle_list,goal_x,goal_y,start_x,start_y,ox,oy):
        plt.clf()
        plt.axis("equal")
        plt.grid(True)
        plt.plot(ox, oy, ".k")
        plt.plot(goal_x,goal_y,"xc")
        plt.plot(start_x,start_y,"xc")
        plt.text(30,65,s='RRT',horizontalalignment='center',color="k",fontsize=20)
        plt.pause(2)
        for a in obstacle_list:
            plt.plot(a[0],a[1],"hb",ms=30*a[2],)
        for edges in self.edges:
            from_node_x=edges.start_node.x_cor
            from_node_y=edges.start_node.y_cor
            to_node_x=edges.end_node.x_cor
            to_node_y=edges.end_node.y_cor
            plt.plot([from_node_x,to_node_x],[from_node_y,to_node_y],"-g")
            plt.pause(0.4)
        
        plt.axis([0, 60, 0, 60])
        plt.grid(True)
        plt.pause(0.0000000001)
        
        #plt.axis("equal")
        return True
      
    def find_nearest_node(self,from_x,from_y,obstacle_list,goal_x,goal_y):
        smallest=(10000,None)
        for nodes in self.nodes:
            d=math.sqrt((from_x-nodes.x_cor)**2+(from_y-nodes.y_cor)**2)
            #print(from_x,from_y)
            if(d<smallest[0]):
                smallest=(d,nodes)
        #calculate the angle of the random coor with nearest node
        theta = math.atan2(from_y-smallest[1].y_cor,from_x-smallest[1].x_cor)
        scaled_x=smallest[1].x_cor+self.step_dist*math.cos(theta)
        scaled_y=smallest[1].y_cor+self.step_dist*math.sin(theta)
        #print(scaled_x,scaled_y,from_x,from_y,"---------")
        if(self.collision_check(scaled_x,scaled_y,obstacle_list,smallest[1])==True):
            self.add_edge(smallest[1].x_cor,smallest[1].y_cor,scaled_x,scaled_y,2)
            #print(scaled_x,scaled_y)
            self.path[(scaled_x,scaled_y)]=(smallest[1].x_cor,smallest[1].y_cor)
            d=math.sqrt((scaled_x-goal_x)**2+(scaled_y-goal_y)**2)
            if(d<self.step_dist):
                #print(goal_x,goal_y,"******")
                self.path[(goal_x,goal_y)]=(scaled_x,scaled_y)
                self.add_edge(scaled_x,scaled_y,goal_x,goal_y,2)
                print("Bullseye ;")
                return True
        
    def collision_check(self,from_x,from_y,obstacle_list,nodes):
        for a in obstacle_list:
            d_node=math.sqrt((from_x-nodes.x_cor)**2+(from_y-nodes.y_cor)**2)
            d_obs=math.sqrt((from_x-a[0])**2+(from_y-a[1])**2)
            #print(d_obs,a[2])
            if(d_obs<a[2]*self.cost_radius):
                #print(from_x,from_y,a[0],a[1],"-------")
                return False
        return True
        
   
def main():
    ox=[]
    oy=[]
    goal_x=35
    goal_y=45
    start_x=1
    start_y=1
    goal_radius=3
    plt.clf()
    #plotting the simulation area
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
    
    #obstacles that will expanded as circles
    obstacleList = [
        (20,30, 3),
        (10,50, 1),
        (20, 40, 2),
        (30, 20, 2),
        (40, 20, 1),
        (40, 10, 1),
        (50,10, 1),
        (45, 55, 1),
        (50, 47, 1),
        (9, 11, 1),
        (30, 30, 2),
        ]
    
    rrt=RRT(step_dist=3,cost_radius=4)
    rrt.set_root(start_x,start_y)
    
    
    while 1:
        from_x=random.uniform(0,60)
        from_y=random.uniform(0,60)
        goal_reach=rrt.find_nearest_node(from_x,from_y,obstacleList,goal_x,goal_y)
        if(goal_reach==True):
            break
    rrt.draw_tree(obstacleList,goal_x,goal_y,start_x,start_y,ox,oy)
    #drawing the final path
    val=(0,0)
    while((val[0],val[1])!=(start_x,start_y)):
        val=rrt.path[(goal_x,goal_y)]
        #draw path
        plt.plot([val[0],goal_x],[val[1],goal_y],"-r")
        plt.pause(1)
        goal_x=val[0]
        goal_y=val[1]
        

    """
    rrt.add_edge(10,10,15,30,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(15,30,50,40,math.sqrt((10-40)**2+(20-30)**2))
    rrt.add_edge(10,10,15,30,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(50,40,44,33,math.sqrt((10-20)**2+(20-30)**2))
    rrt.add_edge(15,30,5,4,math.sqrt((10-20)**2+(20-30)**2))
    rrt.draw_tree()"""
    
if __name__ == '__main__':
    main()   
    
    