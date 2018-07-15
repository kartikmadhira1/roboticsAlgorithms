"""
An implementation of the Extended Kalman Filter.

@author: kartikmadhira
"""

def update(mean1, var1, mean2, var2):
    new_mean =(mean1*var1+mean2*var2)/(var1+var2)
    new_var =1/((1/(var1)+(1/(var2))))
    return [new_mean, new_var]

print (update(10.,8.,13., 2.))