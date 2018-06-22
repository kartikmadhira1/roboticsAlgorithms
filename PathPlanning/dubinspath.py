
"""
Implementation of Dubins Paths for nonholonomic robots. 

References:
    1) https://gieseanw.files.wordpress.com/2012/10/dubins.pdf (Amazing doc!)

@author: kartikmadhira
"""

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import math


import math


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


def lr_generate(vector,min_radius):
    x,y,theta=vector
    left_angle=theta-math.pi/2
    right_angle=-math.pi/2-theta
    
    left_circle_x=x+min_radius*math.cos(left_angle)
    left_circle_y=y+min_radius*math.sin(left_angle)
    right_circle_x=x+min_radius*math.cos(right_angle)
    right_circle_y=y-min_radius*math.sin(right_angle)
    
    return left_circle_x,left_circle_y,right_circle_x,right_circle_y

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

    #generate the tangent points
    start_tan,goal_tan=tan_coor((s_right_x,s_right_y),(g_left_x,g_left_y),min_radius,min_radius)
    
    ang_1,s_ang_1,e_ang_1=cal_angle((start[0],start[1]),(start_tan[0],start_tan[1]),(s_right_x,s_right_y),'r',min_radius)
    ang_2,s_ang_2,e_ang_2=cal_angle((start[0],start[1]),(start_tan[2],start_tan[3]),(s_right_x,s_right_y),'r',min_radius)
    ang_3,s_ang_3,e_ang_3=cal_angle((goal[0],goal[1]),(goal_tan[0],goal_tan[1]),(g_left_x,g_left_y),'l',min_radius)
    ang_4,s_ang_4,e_ang_4=cal_angle((goal[0],goal[1]),(goal_tan[2],goal_tan[3]),(g_left_x,g_left_y),'l',min_radius)

    switch=False
    if(start[2]-(ang_1+ang_3)==goal[2]):
        print('1')
        s_ang=ang_1
        g_ang=ang_3
        line_s_x,line_s_y=start_tan[0],start_tan[1]
        line_g_x,line_g_y=goal_tan[0],goal_tan[1]
        s_st_ang,s_e_ang=s_ang_1,e_ang_1
        g_st_ang,g_e_ang=s_ang_3,e_ang_3
        switch=True
        
    elif(start[2]-(ang_1+ang_4)==goal[2]):
        print('2')
        s_ang=ang_1
        g_ang=ang_4
        line_s_x,line_s_y=start_tan[0],start_tan[1]
        line_g_x,line_g_y=goal_tan[2],goal_tan[3]
        s_st_ang,s_e_ang=s_ang_1,e_ang_1
        g_st_ang,g_e_ang=s_ang_4,e_ang_4
        switch=True
    
    elif(start[2]-(ang_2+ang_3)==goal[2]):
        print('3')
        s_ang=ang_2
        g_ang=ang_3
        line_s_x,line_s_y=start_tan[2],start_tan[3]
        line_g_x,line_g_y=goal_tan[0],goal_tan[1]
        s_st_ang,s_e_ang=s_ang_2,e_ang_2
        g_st_ang,g_e_ang=s_ang_3,e_ang_3
        switch=True
    
    elif(start[2]-(ang_2+ang_4)==goal[2]):
        print('4')
        s_ang=ang_2
        g_ang=ang_4
        line_s_x,line_s_y=start_tan[2],start_tan[3]
        line_g_x,line_g_y=goal_tan[2],goal_tan[3]
        s_st_ang,s_e_ang=s_ang_2,e_ang_2
        g_st_ang,g_e_ang=s_ang_4,e_ang_4
        switch=True
    cost=(s_ang/360)*2*math.pi+(g_ang/360)*2*math.pi+math.sqrt((line_s_x-line_g_x)**2+(line_s_y-line_g_y)**2)
    if(switch==True):
        return (cost,True,(s_st_ang,s_e_ang,s_right_x,s_right_y),(g_st_ang,g_e_ang,g_left_x,g_left_y))


def main():
    min_radius=5
    fig=plt.figure(1)
    ax=fig.add_subplot(1,1,1)
    ax.axis('scaled')
    ax.axis([0,50,0,50])
    
    #intitial point
    start=(10,10,math.pi/2)
    goal=(40,30,-math.pi)
    
    ax.arrow(start[0],start[1],10*math.cos(start[2]),10*math.sin(start[2]),head_length=3,head_width=3,fc='r', ec='k')
    ax.arrow(goal[0],goal[1],10*math.cos(goal[2]),10*math.sin(goal[2]),head_length=3,head_width=3,fc='r', ec='k')
    
    x=RSL(start,goal,min_radius)
    
    arc1=mpatches.Arc([x[2][2], x[2][3]], 10, 10, angle=0, theta1=x[2][1]*180/math.pi, theta2=x[2][0]*180/math.pi,color="green")
    ax.add_patch(arc1)
    arc2=mpatches.Arc([x[3][2], x[3][3]], 10, 10, angle=0, theta1=x[3][1]*180/math.pi, theta2=x[3][0]*180/math.pi,color="green")
    ax.add_patch(arc2)


if __name__ == '__main__':
    main()

