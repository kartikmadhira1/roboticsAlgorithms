#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 29 16:46:06 2019

@author: kartikmadhira
"""

"""
Creating a Dijkstra's algorithm visualistion using priority queue(Heap)
"""
import matplotlib.pyplot as plt
import heapq as heap
import math
import argparse

animation=True

def motionModel():
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


def dijkstra(startX,startY,goalX,goalY,visitMap):
    #heap initialize
    h=[]
    #dictionary that'll store final path
    finalPath=dict()
    currentList=[]
    #every node in the heap will be of the form:
    #(cost,x,y,last node)
    heap.heappush(h,(0,startX,startY,-1,-1))
    motion=motionModel()
    map_=visitMap
    currX=[]
    currY=[]
    # map_=build_costmap(ox,oy,robot_reach).
    while 1:
        current=heap.heappop(h)
        currentList.append(current)
        #finalPath[(current[3],current[4])]
        #print(current[1],current[2])
        currX.append(current[1])
        currY.append(current[2])

        if animation:
            if len(currentList) % 100 == 0:
                plt.plot(currX,currY,"xr")
                plt.pause(0.01)
        if(current[1]==goalX and current[2]==goalY):
            plt.plot(currX,currY,".r")
            print("Bullseye ;)")
            break
        if len(currentList) == 0:
            plt.plot(currX,currY,".r")
            plt.pause(0.001)
        #adding all jumps to the heap
        for i in range(len(motion)):
            if(map_[current[1]+motion[i][0]][current[2]+motion[i][1]]==False):
                heap.heappush(h,(current[0]+motion[i][2],current[1]+motion[i][0],current[2]+motion[i][1],current[1],current[2]))
                #dictionary ---> (current_node) ***mapped to***** (parent_node)
                finalPath[(current[1]+motion[i][0],current[2]+motion[i][1])]=(current[1],current[2])
                map_[current[1]+motion[i][0]][current[2]+motion[i][1]]=True
        if(len(h)==0):
            print('No path is possible in these configurations')
            return -1
    val=(0,0)
    pathList=[]
    #final path tracking
    print("Tracing the path:")
    valX=[]
    valY=[]
    while ((goalX,goalY)!=(startX,startY)):
        val=finalPath[(goalX,goalY)]
        valX.append(val[0])
        valY.append(val[1])

        #plt.plot(val[0],val[1],"xc")
        pathList.append([val[0],val[1]])
        #plt.pause(0.0000001)
        goalX=val[0]
        goalY=val[1]
    plt.plot(valX,valY,'.c')
    plt.pause(15)
    return pathList 
        
import time
def main():
    start=time.time()
    Parser = argparse.ArgumentParser()    
    Parser.add_argument('--startx',type=int, default=21, help='The start x coordinate  in the range [1,249] for both (x,y)')
    Parser.add_argument('--starty',type=int, default=21, help='The start y coordinate  in the range [1,249] for both (x,y)')
    Parser.add_argument('--endx',type=int, default=230, help='The goal y coordinate  in the range [1,249] for both (x,y)')
    Parser.add_argument('--endy',type=int, default=130, help='The goal y coordinate  in the range [1,249] for both (x,y)')
    Parser.add_argument('--typeRobot',type=bool, default=1, help='1 for rigid robot, 0 for point robot')
    Parser.add_argument('--resolution',type=int, default=1, help='resolution of the map')
    Parser.add_argument('--robotRadius',type=int, default=5, help='radius of the robot')
    Parser.add_argument('--clearance',type=int, default=0, help='clearance of the robot')

   
    
    Args = Parser.parse_args()
    startx = Args.startx
    endx=Args.endx
    starty = Args.starty
    endy=Args.endy
    typeRobot=Args.typeRobot
    robotRadius=Args.robotRadius
    clearance=Args.clearance
    robotRadius=robotRadius+clearance
    startX=startx
    startY=starty
    goalX=endx
    goalY=endy
    if(startX<1 or startX>249):
        print('coordinates are out of bounds, enter valid coordinates')
        return -1
    if(goalX<1 or goalX>249):
        print('coordinates are out of bounds, enter valid coordinates')
        return -1
    if(startY<1 or startY>149):
        print('coordinates are out of bounds, enter valid coordinates')
        return -1
    if(goalY<1 or goalY>149):
        print('coordinates are out of bounds, enter valid coordinates')
        return -1
    print("Start at :",startX,startY)
    pathX=[]
    pathY=[]
    pathTypeX=[]
    pathTypeY=[]
    visitMap=[[False for _ in range(151)]for _ in range(251)]

    for i in range(250):
        pathX.append(i)
        pathY.append(0.0)
        visitMap[i][0]=True

    for i in range(150):
        pathX.append(250.0)
        pathY.append(i) 
        #print(i)
        visitMap[250][i]=True

    for i in range(250):
        pathX.append(i)
        pathY.append(150.0)
        visitMap[i][150]=True
        
    for i in range(150):
        pathX.append(0.0)
        pathY.append(i)
        visitMap[0][i]=True

    for x in range(250):
        for y in range(150):
            if(typeRobot):
                if((x-140)**2*((robotRadius+6)**2)+(y-120)**2*((robotRadius+15)**2)-((robotRadius+15)**2)*((robotRadius+6)**2)<=0):         
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
    
                if((((x-190)**2)+((y-130)**2)-((robotRadius+15)**2))<0):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
    
                if(-41*x-25*y+(135.88-robotRadius)*48.02<=0 and 38*x+23*y-(robotRadius+192.04)*44.42<=0 and 37*x-20*y-(145.06+robotRadius)*42.05<=0 and -y+(-robotRadius+15)<=0 and (4*x+38*y-2628-robotRadius*(math.sqrt(4**2+38**2))<=0 or -38*x+7*y+(-robotRadius+150.88)*38.64<=0)):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
    
                if(((y-(robotRadius+112.50))<0) and
                   ((x-(robotRadius+100))<0) and
                   ((-y+(-robotRadius+67.5)<0)) and
                   ((-x+(-robotRadius+50)<0))):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
                if(y-robotRadius<0):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
                if(x-robotRadius<0):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
                if(y-(-robotRadius+150)>0):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
                if(x-(-robotRadius+250)>0):
                    pathX.append(x)
                    pathY.append(y)
                    visitMap[x][y]=True
                    
            if(y-112.5<=0 and x-100<=0 and -y+67.5<=0 and -x+50<=0):
                pathTypeX.append(x)
                pathTypeY.append(y)
                visitMap[x][y]=True

            if((x-190)**2+(y-130)**2-15**2<=0):
                pathTypeX.append(x)
                pathTypeY.append(y)
                visitMap[x][y]=True

            if((x-140)**2*(6**2)+(y-120)**2*(15**2)-(15**2)*(6**2)<=0):
                pathTypeX.append(x)
                pathTypeY.append(y)
                visitMap[x][y]=True

            if(-41*x-25*y+6525<=0 and 38*x+23*y-8530<=0 and 37*x-20*y-6101<=0 and -y+15<=0 and (4*x+38*y-2628<=0 or -38*x+7*y+5830<=0)):
                pathTypeX.append(x)
                pathTypeY.append(y)
                visitMap[x][y]=True

    if(visitMap[startX][startY]==True):
        print('The start node is in obstacle space, please enter valid start configuration')
        return -1
    if(visitMap[goalX][goalY]==True):
        print('The goal node is in obstacle space, no path is possible in this goal configuration')
        return -1 
            
    #if(visitMap[startX][]==True)        
    fig,ax=plt.subplots()

    if animation:
        ax.plot(startX,startY,"xr")
        ax.plot(goalX,goalY,"xr")
        ax.plot(pathX,pathY,".b")
        ax.plot(pathTypeX,pathTypeY,".k")

        #ax.grid(True)
        ax.set_ylim([0,160])
        ax.set_xlim([0,300])
        ax.axis("equal")
    path=dijkstra(startX,startY,goalX,goalY,visitMap)
    end=time.time()
    #print("Time taken :", abs(start-end),"s")
if __name__ == '__main__':
    main()    
