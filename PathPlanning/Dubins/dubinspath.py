
"""
Implementation of Dubins Paths for nonholonomic robots. 

References:
    1) https://gieseanw.files.wordpress.com/2012/10/dubins.pdf (Amazing doc!)

@author: kartikmadhira
"""

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import math
import random


#this generates tangent coordinates for each of the circle involved.
def tan_coor(center_first,center_second,radius1,radius2):
    x1,y1=center_first
    x2,y2=center_second
    
    x11=x2-x1
    y11=y2-y1
    
    D=(x1-x2)**2+(y1-y2)**2
    #calculating the angle(c) between line passing through centres and tangent point
    cos_theta=(radius1-radius1)/D
    sin_theta=math.sqrt(D-(radius1-radius2)**2)/D
    
    #left unit vector calculation 
    left_ux=-x11*cos_theta-y11*sin_theta
    left_uy=-y11*cos_theta+x11*sin_theta
    
    #right unit vector calculation 
    right_ux=-x11*cos_theta+y11*sin_theta
    right_uy=-y11*cos_theta-x11*sin_theta
    
    left_x_1=x1+left_ux*radius1
    left_y_1=y1+left_uy*radius1
    
    left_x_2=x2+left_ux*radius2
    left_y_2=y2+left_uy*radius2
    
    right_x_1=x1+right_ux*radius1
    right_y_1=y1+right_uy*radius1
    
    right_x_2=x2+right_ux*radius2
    right_y_2=y2+right_uy*radius2
    
    return ([left_x_1,left_y_1,right_x_1,right_y_1],[left_x_2,left_y_2,right_x_2,right_y_2])

#Generates coordinates for the left and right traversing circles areound initial point.
def lr_generate(vector,min_radius):
    x,y,theta=vector
    left_angle=theta-math.pi/2
    right_angle=-math.pi/2-theta
    
    left_circle_x=x+min_radius*math.cos(left_angle)
    left_circle_y=y+min_radius*math.sin(left_angle)
    right_circle_x=x+min_radius*math.cos(right_angle)
    right_circle_y=y-min_radius*math.sin(right_angle)
    
    return left_circle_x,left_circle_y,right_circle_x,right_circle_y

#Generates tangent angles to traverse by checking tangent points and initial point.
def cal_angle(start,tangent,center,turn,min_radius):
    
    vector1_x=start[0]-center[0]
    vector1_y=start[1]-center[1]
    vector2_x=tangent[0]-center[0]
    vector2_y=tangent[1]-center[1]
 
    
    theta=math.atan2(vector2_y,vector2_x)-math.atan2(vector1_y,vector1_x)
    if(theta<0 and turn=='l' ):
        theta=theta+2*math.pi
    elif(theta>0 and turn=='r'):
        theta=theta-2*math.pi
    return abs(theta),math.atan2(vector1_y,vector1_x),math.atan2(vector2_y,vector2_x)



def RSL(start,goal,min_radius):
    
    #generate only the left of goal and right of start circle
    s_right_x,s_right_y,left_x,left_y=lr_generate(start,min_radius)
    right_x,right_y,g_left_x,g_left_y=lr_generate(goal,min_radius)
    
    start_tan,goal_tan=tan_coor((s_right_x,s_right_y),(g_left_x,g_left_y),min_radius,min_radius)
    #generate the tangent points
    ang_1,s_ang_1,e_ang_1=cal_angle((start[0],start[1]),(start_tan[0],start_tan[1]),(s_right_x,s_right_y),'r',min_radius)
    ang_2,s_ang_2,e_ang_2=cal_angle((goal[0],goal[1]),(goal_tan[2],goal_tan[3]),(g_left_x,g_left_y),'l',min_radius)
    
    cost=((abs(s_ang_1-e_ang_1)/360)+(abs(s_ang_2-e_ang_2)/360))*2*math.pi*min_radius+(start_tan[0]-goal_tan[2])**2+(start_tan[1]-goal_tan[3])**2
    
    return (cost,(s_right_x,s_right_y,g_left_x,g_left_y),(s_ang_1,e_ang_1,start_tan[0],start_tan[1]),(s_ang_2,e_ang_2,goal_tan[2],goal_tan[3]),'RSL')
  
def LSL(start,goal,min_radius):
    
    #generate only the left of goal and right of start circle
    s_right_x,s_right_y,left_x,left_y=lr_generate(start,min_radius)
    right_x,right_y,g_left_x,g_left_y=lr_generate(goal,min_radius)
    
    start_tan,goal_tan=tan_coor((left_x,left_y),(g_left_x,g_left_y),min_radius,min_radius)
    #generate the tangent points
    ang_1,s_ang_1,e_ang_1=cal_angle((start[0],start[1]),(start_tan[2],start_tan[3]),(left_x,left_y),'l',min_radius)
    ang_2,s_ang_2,e_ang_2=cal_angle((goal[0],goal[1]),(goal_tan[2],goal_tan[3]),(g_left_x,g_left_y),'l',min_radius)
    
    cost=((abs(s_ang_1-e_ang_1)/360)+(abs(s_ang_2-e_ang_2)/360))*2*math.pi*min_radius+(start_tan[2]-goal_tan[2])**2+(start_tan[3]-goal_tan[3])**2
    
    return (cost,(left_x,left_y,g_left_x,g_left_y),(e_ang_1,s_ang_1,start_tan[2],start_tan[3]),(s_ang_2,e_ang_2,goal_tan[2],goal_tan[3]),'LSL')    

def RSR(start,goal,min_radius):
    
    #generate only the left of goal and right of start circle
    s_right_x,s_right_y,left_x,left_y=lr_generate(start,min_radius)
    right_x,right_y,g_left_x,g_left_y=lr_generate(goal,min_radius)
    
    start_tan,goal_tan=tan_coor((s_right_x,s_right_y),(right_x,right_y),min_radius,min_radius)
    #generate the tangent points
    ang_1,s_ang_1,e_ang_1=cal_angle((start[0],start[1]),(start_tan[0],start_tan[1]),(s_right_x,s_right_y),'r',min_radius)
    ang_2,s_ang_2,e_ang_2=cal_angle((goal[0],goal[1]),(goal_tan[0],goal_tan[1]),(right_x,right_y),'r',min_radius)
    
    cost=((abs(s_ang_1-e_ang_1)/360)+(abs(s_ang_2-e_ang_2)/360))*2*math.pi*min_radius+(start_tan[0]-goal_tan[0])**2+(start_tan[1]-goal_tan[1])**2
    
    return (cost,(s_right_x,s_right_y,right_x,right_y),(s_ang_1,e_ang_1,start_tan[0],start_tan[1]),(e_ang_2,s_ang_2,goal_tan[0],goal_tan[1]),'RSR')

def LSR(start,goal,min_radius):
    
    #generate only the left of goal and right of start circle
    s_right_x,s_right_y,left_x,left_y=lr_generate(start,min_radius)
    right_x,right_y,g_left_x,g_left_y=lr_generate(goal,min_radius)
    
    start_tan,goal_tan=tan_coor((left_x,left_y),(right_x,right_y),min_radius,min_radius)
    #generate the tangent points
    ang_1,s_ang_1,e_ang_1=cal_angle((start[0],start[1]),(start_tan[2],start_tan[3]),(left_x,left_y),'l',min_radius)
    ang_2,s_ang_2,e_ang_2=cal_angle((goal[0],goal[1]),(goal_tan[0],goal_tan[1]),(right_x,right_y),'r',min_radius)
    
    cost=((abs(s_ang_1-e_ang_1)/360)+(abs(s_ang_2-e_ang_2)/360))*2*math.pi*min_radius+(start_tan[2]-goal_tan[0])**2+(start_tan[3]-goal_tan[1])**2
    
    return (cost,(left_x,left_y,right_x,right_y),(e_ang_1,s_ang_1,start_tan[2],start_tan[3]),(e_ang_2,s_ang_2,goal_tan[0],goal_tan[1]),'LSR')


def main():
    min_radius=5
    #initiate all the functions for paths to be maintained
    paths=[LSL,RSR,RSL,LSR]
    #generate n number of paths where we randomly intiate start and goal configurations
    for _ in range(20):
        fig=plt.figure(1)
        ax=fig.add_subplot(1,1,1)
        ax.axis('scaled')
        ax.axis([0,100,0,100])
        #start random coordinates.
        s_x=random.randint(10,80)
        s_y=random.randint(10,80)
        #goal random coordinates.
        g_x=random.randint(10,80)
        g_y=random.randint(10,80)
        #start and goal angles wrt origin.
        s_angle=random.uniform(0,2)
        g_angle=random.uniform(0,2)
        start=(s_x,s_y,s_angle*math.pi)
        goal=(g_x,g_y,g_angle*math.pi)
        d=math.sqrt((s_x-g_x)**2+(g_x-g_y)**2)
        #check for circles being too close to each other
        if(d>=(min_radius*4.5)):
            #start arrow
            arrow1=ax.arrow(start[0],start[1],10*math.cos(start[2]),10*math.sin(start[2]),head_length=3,head_width=3,fc='r', ec='k',label='Start Configuration' )
            #goal arrow
            arrow2=ax.arrow(goal[0],goal[1],10*math.cos(goal[2]),10*math.sin(goal[2]),head_length=3,head_width=3,fc='b', ec='k',label='End Configuration')
            plt.legend([arrow1,arrow2,],['Start configuration','Goal configuration'])
            #bubble out the path with the least cost.
            cost_final=math.inf
            for path in paths:
                ret_val=path(start,goal,min_radius)
                print(ret_val[0],ret_val[4])
                if(abs(ret_val[0])<cost_final):
                    final_path=ret_val
                    cost_final=abs(ret_val[0])
            plt.legend([arrow1,arrow2,final_path[0],],['Start configuration','Goal configuration','Cost to goal:',])
            plt.text(40,90,s=final_path[4],horizontalalignment='center',color="k",fontsize=15)
            h=final_path
            #h=RSL(start,goal,min_radius)
            print("Final path is: ",final_path[4])
            arc1=mpatches.Arc([h[1][0], h[1][1]], min_radius*2, min_radius*2, angle=0, theta1=h[2][1]*180/math.pi, theta2=h[2][0]*180/math.pi,color="green")
            ax.add_patch(arc1)
            arc2=mpatches.Arc([h[1][2], h[1][3]], min_radius*2, min_radius*2, angle=0, theta1=h[3][1]*180/math.pi, theta2=h[3][0]*180/math.pi,color="green")
            plt.plot([h[2][2],h[3][2]],[h[2][3],h[3][3]],'g-',linewidth=1)
            plt.plot(h[3][2],h[3][3],"xr")
            ax.add_patch(arc2)
            plt.pause(3)
            plt.close()

if __name__ == '__main__':
    main()

