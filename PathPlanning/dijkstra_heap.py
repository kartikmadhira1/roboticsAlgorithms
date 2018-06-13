"""
Creating a Dijkstra's algorithm visualistion using priority queue(Heap)
Note: This work is loosely based on AtsushiAkai's repo on PythonRobotics with modifications.
(https://github.com/AtsushiSakai/PythonRobotics)

"""
import matplotlib.pyplot as plt
import heapq as heap
import math

show_animation=True

def build_costmap(ox,oy,robot_reach):
    #this is where we build a map with a 2D array
    visit_map=[[False for _ in range(60)]for _ in range(60)]
    #for every point in the map, calculate the distance from each of the 
    #obstacle/border points in the map. If the distance is less than the 
    #robot's reach, replace it with True val.
    for x in range(60):
        for y in range(60):
            for obs_x,obs_y in zip(ox,oy):
                #calculating the euclidean distance
                d=math.sqrt((obs_x-x)**2+(obs_y-y)**2)
                if(d<=robot_reach):
                    visit_map[x][y]=True

    return visit_map

def motion_model():
    #steps that the robot will take for every queue element iteration
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]
    return motion


def dijkstra(start_x,start_y,goal_x,goal_y,ox,oy,robot_reach):
    #heap initialize
    h=[]
    #dictionary that'll store 
    final_path=dict()
    current_list=[]
    #every node in the heap will be of the form:
    #(cost,x,y,last node)
    heap.heappush(h,(0,start_x,start_y,-1,-1))
    motion=motion_model()
    map_=build_costmap(ox,oy,robot_reach)
    while 1:
        current=heap.heappop(h)
        current_list.append(current)
        #final_path[(current[3],current[4])]
        #print(current[1],current[2])
        if show_animation:
            plt.plot(current[1],current[2],"xr")
            if len(current_list) % 10 == 0:
                plt.pause(0.001)
        if(current[1]==goal_x and current[2]==goal_y):
            print("Bullseye ;)")
            break
        
        #adding all jumps to the heap
        for i in range(len(motion)):
            if(map_[current[1]+motion[i][0]][current[2]+motion[i][1]]==False):
                heap.heappush(h,(current[0]+motion[i][2],current[1]+motion[i][0],current[2]+motion[i][1],current[1],current[2]))
                #dictionary ---> (current_node) ***mapped to***** (parent_node)
                final_path[(current[1]+motion[i][0],current[2]+motion[i][1])]=(current[1],current[2])
                map_[current[1]+motion[i][0]][current[2]+motion[i][1]]=True
    val=(0,0)
    path_list=[]
    #final path tracking
    print("Tracing the path:")
    while ((goal_x,goal_y)!=(start_x,start_y)):
        val=final_path[(goal_x,goal_y)]
        plt.plot(val[0],val[1],"xc")
        path_list.append([val[0],val[1]])
        plt.pause(0.0001)
        goal_x=val[0]
        goal_y=val[1]
    return path_list 
        
import time
def main():
    start=time.time()
    start_x=10
    start_y=10
    goal_x=50
    goal_y=50
    robot_reach=1
    print("Start at :",start_x,start_y)
    ox=[]
    oy=[]
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
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
   
    if show_animation:
        plt.plot(start_x,start_y,"xr")
        plt.plot(goal_x,goal_y,"xr")
        plt.plot(ox,oy,"xc")
        plt.grid(True)
        plt.axis("equal")

    path=dijkstra(start_x,start_y,goal_x,goal_y,ox,oy,robot_reach)
    end=time.time()
    print("Time taken :", abs(start-end),"s")
if __name__ == '__main__':
    main()    