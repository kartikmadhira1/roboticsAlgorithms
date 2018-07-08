"""
Pure Pursuit tracking with Proportional-Integral-Derivative control

@author: kartikmadhira
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

def point_locate(x,y,path,L):
    point=()
    for a in zip(path[0],path[1]):
        if(math.sqrt((a[0]-x)**2+(a[1]-y)**2)<L**2):
            point=(a[0],a[1])
    return point
            
def main(): 
    #L is the Lookahead distance
    L=0.5
    #gains
    kp=0.0001
    ki=0.00001
    kd=0.001
    #start configuration
    start=(0.2,5.2,0)
    #actual path trace
    p_x=np.linspace(0,10,num=1000)
    path_x=np.array(p_x).tolist()
    p_y=np.convolve(1+np.sin(p_x),5)
    path_y=np.array(p_y).tolist()
    #traversed trace list
    ox,oy=[],[]
    #time stamps of linear spacings for plot
    dt=np.linspace(0, 1000,num=10000)
    curr_x=start[0]
    curr_y=start[1]
    curr_ang=start[2]
    theta=0
    Integral=0
    prev_t=-0.001
    
    for a in dt:
        fig=plt.figure(1)
        ax=fig.add_subplot(1,1,1)
        ax.axis('scaled')
        ax.axis([0,11,0,11])
        plt.plot(path_x,path_y)
        plt.plot(curr_x,curr_y,"xg")
        plt.plot(ox,oy)
        #locate the points in the lookahead thats closest and farthest on the line.
        p=point_locate(curr_x,curr_y,(path_x,path_y),L)
        #error estimate from the angle lookahead and the current angle
        theta+=math.atan2(p[1]-curr_y,p[0]-curr_x)-curr_ang
        #drive integral for straight paths when erroe becomes zero
        Integral+=abs(ki*theta*(a-prev_t))
        #drive derivate on sharp turns
        Derivative=kd*(theta/(a-prev_t))
        #proportional to theta(error)
        Proportional=kp*(theta)
        #PID addition
        PID_vel=Proportional+Integral
        
        #if values reach a certain high speed limit then saturate the gain
        if(PID_vel>0.003):
            #saturate gain
            PID_vel=0.003
      
        #x and y direction vels for animation(2-D plots)
        utx=(PID_vel*math.cos(theta))
        uty=(PID_vel*math.sin(theta))
        #coordinates w.r.t time dt
        curr_x=curr_x+utx*a
        ox.append(curr_x)
        curr_y=curr_y+uty*a
        oy.append(curr_y)
        plt.plot(p[0],p[1],"xr")
        ax.arrow(start[0],start[1],0.1*math.cos(start[2]),0.1*math.sin(start[2]),head_width=0.1,head_length=0.09)
        plt.text(5.5,10,s="Time(s):"+str(round(a,2)),horizontalalignment='center',color="k",fontsize=11)
        plt.text(5.5,10.5,s="Error:"+str(round(PID_vel,5)),horizontalalignment='center',color="k",fontsize=11)
        plt.plot([curr_x,p[0]],[curr_y,p[1]],"r-")
        plt.pause(0.1)
        plt.close()
        prev_t=a
        curr_ang=theta
        if(((curr_x-path_x[-1])**2+(curr_y-path_y[-1]**2))<1):
            break
    plt.close()

if __name__=='__main__':
    main()