"""
An implementation of the Extended Kalman Filter.

@author: kartikmadhira
"""

#This is a sympy snippet to calculate Jacobian for measurement and observation models
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