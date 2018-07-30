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
    iv. System input noise model
2. Initialize input,motion and observation models.

"""

import math
import numpy as np
import random
import matplotlib.pyplot as plt

#Covariance matrices, assuming each state is independant of each other.
#state(motion) covariance matrix
Q=np.matrix(np.diag([0.2,0.2,math.radians(1.0),0.2]))
#observation covariance matrix
R=np.matrix(np.diag([0.2,0.2]))
#input noise covariance matrix
U=np.matrix(np.diag([0.24,0.24]))

def input_model():
    v=1 #m/s
    omega=0.4 #rad/s
    #adding noise to the input u
    u=np.matrix([v+random.random()*U[0,0],omega+math.radians(random.random()*U[1,1])]).T
    return u

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
    F=np.matrix([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,0]])
    #we need a 4*1 matrix and hence B will be (4*2) and u is (2*1) 
    B=np.matrix([[0.1*u[0,0]*math.cos(x[2,0]),0],
                 [0.1*u[0,0]*math.sin(x[2,0]),0],
                 [0,0.1],
                 [1.0,0.0]])
    x=F*x+B*u
    return x

def observation_model(x):
    #z=H*x
    #H is (2*4) and x is (4*1) 
    """
    H=np.matrix([1,0,0,0],
                [0,1,0,0])
    """
    z=np.matrix([x[0,0]+random.random()*R[0,0], x[1,0]+random.random()*R[1,1]])
    #adding noise to the sensor reporting coordinates.(gps)
    return z

def jacobianF(x,u):
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
    jF=np.matrix([[1,0,-0.1*u[0,0]*math.sin(x[2,0]),0.1*math.cos(x[2,0])],
                  [0,1,0.1*u[0,0]*math.cos(x[2,0]),0.1*math.sin(x[2,0])],
                  [0,0,1,0],
                  [0,0,0,1]])
    return jF


def jacobianH(x):
    # jacobian of observation Model
    """
    import sympy
    from sympy.abc import alpha, x, y, v, w, R, theta,omega
    from sympy import symbols, Matrix
    sympy.init_printing(use_latex="mathjax", fontsize='18pt')
    
    fxu = Matrix([[x],
                  [y]])
                
    F = fxu.jacobian(Matrix([x, y, theta,v]))
    F
    """
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH

def main():
    x=np.matrix(np.zeros((4,1))) #x=[x y theta v](transpose)
    #initial covariance matrix
    P=np.eye(4)
    #input with noise
    noise_u=input_model()
    z_plot=np.matrix([[0,0]])
    while(1):
        #predict
        x_pred=motion_model(x,noise_u)
        F=jacobianF(x,noise_u)
        P_pred=F*P*F.T+Q
        
        #update
        z_pred=observation_model(x_pred)
        #print(z_pred)
        z_plot=np.vstack((z_plot,z_pred))
        plt.cla()
        plt.plot(z_plot[:, 0], z_plot[:, 1], ".g")
        z_curr=x_pred[0:2]
        y=z_curr-z_pred
        H=jacobianH(x_pred)
        K=P_pred*H.T*np.linalg.inv(H*P_pred*H.T+R)
        x_post=x_pred+K*y
        P_post=(np.eye(4)-K*H)*P_pred
        x=x_pred
        P=P_pred
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
        
if __name__=='__main__':
    main()


