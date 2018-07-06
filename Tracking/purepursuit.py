#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Pursuit tracking with Proportional-Integral-Derivative control

@author: kartikmadhira
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patch
import numpy as np
import math

    

def point_locate(x,y,path,L):
    point=()
    for a in zip(path[0],path[1]):
        if(math.sqrt((a[0]-x)**2+(a[1]-y)**2)<L):
            point=(a[0],a[1])
    return point
            

def main(): 
    #L is the Lookahead distance
    L=0.8
    #start configuration
    start=(1,1.5,math.pi/3)
    #actual path trace
    path_x=np.linspace(1,10)
    path_y=2*abs(np.sin(path_x))
    #linear velocity
    Vr=4
    utx=Vr*math.cos(start[2])
    uty=Vr*math.sin(start[2])
    #traversed trace list
    ox,oy=[],[]
    error_x,error_y=0,0
    a=0
    dt=np.linspace(0, 100)
    for a in dt:
        fig=plt.figure(1)
        ax=fig.add_subplot(1,1,1)
        ax.axis('scaled')
        ax.axis([0,5,0,4])
        plt.plot(path_x,path_y)
        p=point_locate(start[0],start[1],(path_x,path_y),L)
        plt.plot(p[0],p[1],"xr")
        ax.arrow(start[0],start[1],0.1*math.cos(start[2]),0.1*math.sin(start[2]),head_width=0.09,head_length=0.09)
        plt.text(2.5,2,s="Time(s):"+str(a),horizontalalignment='center',color="k",fontsize=15)
        plt.pause(1)
        plt.close()
        a+=1
        

if __name__=='__main__':
    main()