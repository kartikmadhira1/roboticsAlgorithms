"""
An implementation of the Extended Kalman Filter.

@author: kartikmadhira
"""

#Flow of the code is as follows:
"""
1. Intialize system with state vector matrix, measurement covariance matrix, observation matrix and Input noise matrix
    i. x=[x y theta v]
    ii. Q -> state or motion covariance matrix.
    iii. R -> Observation covariance matrix
    iv. Input noise model
2. Initialize input,motion and observation models.

"""

import math
import numpy as np


#state(motion) covariance matrix
Q=np.matrix(np.diag([0.2,0.2,math.radians(1.0),0.2]))
#observation covariance matrix
R=np.matrix(np.diag([0.2,0.2]))

#input covariance matrix
U=np.matrix(np.diag([0.24,0.24]))

def input_model():
    v=1 #m/s
    omega=0.4 #rad/s
    u=np.matrix((v,omega)).T
    return u

#reports back us the position so H matrix be upated appropriately.
def observation_model(x):
    #z=H*x
    #H is (2*4) and x is (4*1) 
    H=np.matrix([1 ,0 ,0,0],
                [0,1,0,0])
    
    z=H*x
    return z
    
def motion_model(x,u):
    """
    motion model is:
    x_{t+1}=x_{t}+v*dt*cos(theta)
    y_{t+1}=y_{t}+v*dt*sin(theta)
    theta_{t+1}=omega*dt
    v_{t+1}=v_{t}
    """
    #representing this in the state space form.
    #x=F*x+B*u
    F=np.matrix[[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,0]]
    #we need a 4*1 matrix and hence B will be (4*2) and u is (2*1) 
    B=np.matrix[[0.1*v*math.cos(x[2,0]),0],
                 [0.1*math.sin(x[2,0]),0],
                 [0,0.1],
                 [1.0,0.0]]
    x=F*x+B*u
    return x
    

def main():
    x=np.matrix(np.zeros((4,1))) #x=[x y theta v](transpose)

if __name__=='__main__':
    main()


#This is a sympy snippet to calculate Jacobian for measurement and observation models
#uncomment to know Jacobian equations
"""
import sympy
from sympy.abc import alpha, x, y, v, w, R, theta,omega
from sympy import symbols, Matrix
sympy.init_printing(use_latex="mathjax", fontsize='18pt')

fxu = Matrix([[x+v*time*sympy.cos(theta)],
              [y+v*time*sympy.sin(theta)],
              [theta+omega*time],
              [v]]
                )
F = fxu.jacobian(Matrix([x, y, theta,v]))
F
"""

