#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Implementation of Dubins Paths for nonholonomic robots. 

References:
    1) https://gieseanw.files.wordpress.com/2012/10/dubins.pdf (Amazing doc!)

@author: kartikmadhira
"""

import matplotlib.patches as mpatches

import matplotlib.pyplot as plt
import math
#from pyplot import Figure, subplot

#getting a polar form of the vector
class Vector(object):
    def __init__(self,mag,theta):
        self.mag=mag
        self.theta=theta
        

import math

def rotate_arrow(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point
    angle=angle-math.pi/4
    qx = ox+math.cos(angle)* (px - ox) - math.sin(angle)* (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle)* (py - oy)
    return qx, qy

def rotate_point(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point
    qx = ox-math.cos(angle)* (px - ox) + math.sin(angle)* (py - oy)
    qy = oy - math.sin(angle) * (px - ox) - math.cos(angle)* (py - oy)
    return qx, qy

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


def calc_shortest_path(start,goal):
    
    #generating all left and right circles for each goal and start/initial point
    right_x,right_y,left_x,left_y=lr_generate(start,5)
    g_right_x,g_right_y,g_left_x,g_left_y=lr_generate(goal,5)

    #generating tangent coordinate for each of the circles
    a,b=tan_coor((right_x,right_y),(g_left_x,g_left_y),min_radius,min_radius)
    c,d=tan_coor((left_x,left_y),(g_right_x,g_right_y),min_radius,min_radius)
    """
    a--->right tangent coordinates of initial point
    b--->left tangent coordinates goal left points
    """
    
min_radius=5
fig=plt.figure(1)
ax=fig.add_subplot(1,1,1)
ax.axis('scaled')
ax.axis([0,50,0,50])

#intitial point
start=(10,10,math.pi/3)
goal=(40,30,-math.pi)

arr1=plt.Arrow(10,10,s_plot_x,s_plot_y)
#ax.add_patch(arr1)
#ax.arrow(start[0],start[1],1*s_plot_x,1*s_plot_y,head_length=3,head_width=3,fc='r', ec='k')
#ax.arrow(goal[0],goal[1],0.1*g_plot_x,0.1*g_plot_y,head_length=3,head_width=3,fc='r', ec='k')
ax.arrow(start[0],start[1],10*math.cos(start[2]),10*math.sin(start[2]),head_length=3,head_width=3,fc='r', ec='k')


right_x,right_y,left_x,left_y=lr_generate(start,5)
g_right_x,g_right_y,g_left_x,g_left_y=lr_generate(goal,5)

a,b=tan_coor((right_x,right_y),(g_left_x,g_left_y),min_radius,min_radius)
c,d=tan_coor((left_x,left_y),(g_right_x,g_right_y),min_radius,min_radius)


plt.plot([g_left_x,g_right_x],[g_left_y,g_right_y],"r-")
plt.plot([left_x,right_x],[left_y,right_y],"r-")


circle1=plt.Circle((left_x,left_y),min_radius,fill=False)
circle2=plt.Circle((g_left_x,g_left_y),min_radius,fill=False)
circle3=plt.Circle((right_x,right_y),min_radius,fill=False)
circle4=plt.Circle((g_right_x,g_right_y),min_radius,fill=False)

ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)
ax.add_patch(circle4)


angle,start_angle,end_angle=cal_angle((goal[0],goal[1]),(b[2],b[3]),(g_left_x,g_left_y),'l',min_radius)

angle1,start_angle1,end_angle1=cal_angle((start[0],start[1]),(b[0],b[1]),(right_x,right_y),'r',min_radius)
print(angle*180/math.pi,angle1*180/math.pi)
ax.arrow(goal[0],goal[1],10*math.cos(goal[2]),10*math.sin(goal[2]),head_length=3,head_width=3,fc='r', ec='k')
ax.arrow(goal[0],goal[1],10*math.cos(start[2]+(angle-angle1)),10*math.sin(start[2]+(angle-math.pi-angle1)),head_length=3,head_width=3,fc='r', ec='k')

arc1=mpatches.Arc([g_left_x, g_left_y], 10, 10, angle=0, theta1=end_angle*180/math.pi, theta2=start_angle*180/math.pi,color="green")
ax.add_patch(arc1)



arc=mpatches.Arc([right_x,right_y], 10, 10, angle=0, theta1=end_angle1*180/math.pi, theta2=start_angle1*180/math.pi,color="green")
ax.add_patch(arc)


#plt.plot([-left[0],-left[1]],[-left[1],-left[0]],"r-")

plt.plot(b[2],b[3],"xr")
plt.plot(b[0],b[1],"xr")

plt.plot(a[0],a[1],"xr")
plt.plot(a[2],a[3],"xr")
plt.plot(c[2],c[3],"xr")
plt.plot(c[0],c[1],"xr")
plt.plot(d[0],d[1],"xr")
plt.plot(d[2],d[3],"xr")


































""