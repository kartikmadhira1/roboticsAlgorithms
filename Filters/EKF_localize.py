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
Q=np.matrix(np.diag([0.1, 0.1, math.radians(1.0), 1.0]))**2
#observation covariance matrix
R=np.matrix(np.diag([1.0, math.radians(40.0)]))**2
#input noise covariance matrix
Q_obs = np.diag([0.5, 0.5])**2
U = np.diag([1.0, math.radians(30.0)])**2

def input_model():
    v=1 #m/s
    omega=0.2 #rad/s
    u=np.matrix([v, omega]).T
    return u

def input_model_noise():
    v=1
    omega=0.2
    u=np.matrix([v+np.random.randn()*U[0,0],omega+math.radians(np.random.randn()*U[1,1])]).T
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
    B=np.matrix([[0.1*math.cos(x[2,0]),0],
                 [0.1*math.sin(x[2,0]),0],
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
    z=np.matrix([x[0,0]+np.random.randn()*Q_obs[0,0], x[1,0]+np.random.randn()*Q_obs[1,1]])
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
    jF=np.matrix([[1.0,0.0,-0.1*u[0,0]*math.sin(x[2,0]),0.1*math.cos(x[2,0])],
                  [0.0,1.0,0.1*u[0,0]*math.cos(x[2,0]),0.1*math.sin(x[2,0])],
                  [0.0,0.0,1.0,0.0],
                  [0.0,0.0,0.0,1.0]])
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

def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    x=np.matrix(np.zeros((4,1))) #x=[x y theta v](transpose)
    #initial covariance matrix
    x_wo_filter=np.matrix(np.zeros((4,1)))
    x_actual=np.matrix(np.zeros((4,1)))
    P=np.eye(4)
    #input with noise
    z_plot=np.matrix([[0,0]])
    x_plot=np.matrix([[0,0,0,0]]).T
    x_true=np.matrix([[0,0,0,0]]).T
    while(1):
        #predict
        noise_u=input_model_noise()
        u=input_model()
        #print(noise_u)
        x_actual=motion_model(x_actual,u)
        x_pred=motion_model(x,noise_u)
        x_final=motion_model(x_wo_filter,noise_u)
        x_true=np.hstack((x_true,x_final))
        #print(x_plot[0,:],x_plot[1,:])
        F=jacobianF(x,noise_u)
        P_pred=F*P*F.T+Q
    
        #update
        z_pred=observation_model(x_actual)
        #print(z_pred)
        z_plot=np.vstack((z_plot,z_pred))
        plt.cla()
        plt.plot(z_plot[:, 0], z_plot[:, 1], ".g")
        plt.plot(np.array(x_true[0, :]).flatten(),
                     np.array(x_true[1, :]).flatten(), "-k")
        z_curr=x_pred[0:2]
        y=z_curr-z_pred.T
        H=jacobianH(x_actual)
        K=P_pred*H.T*np.linalg.inv(H*P_pred*H.T+R)
        x_post=x_actual+K*y
        #print(x_post[0,0],x_post[0,1])
        x_plot=np.hstack((x_plot,x_post))
        #print(x_plot[0,:],x_plot[1,:])
        plt.plot(np.array(x_plot[0, :]).flatten(),
                     np.array(x_plot[1, :]).flatten(), "-b")
        P_post=(np.eye(len(x_post))-K*H)*P_pred
        # print(P_post)
        print('\n')
        plot_covariance_ellipse(x_post,P_post)
        x=x_pred
        P=P_pred
        x_wo_filter=x_final
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
        
if __name__=='__main__':
    main()


