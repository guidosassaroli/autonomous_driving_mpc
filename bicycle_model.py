from casadi import *
import numpy as np
import matplotlib.pyplot as plt
from pylab import *
import math


# states
# x1 = x-position in a global coordinate system
# x2 = y-position in a global coordinate system
# x3 = steering angle of front wheels
# x4 = velocity in x-direction
# x5 = yaw angle

def bicycle_model():

    # define structs
    #constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "bicycle_model" 

    #parameters 

    l_wb = 4    # wheel base [m]

    
    # x : states
    x1 = SX.sym('x1') # x position in a global coordinate system
    x2 = SX.sym('x2') # y position in a global coordinate system
    x3 = SX.sym('x3') # delta: steering angle of front wheels
    x4 = SX.sym('x4') # velocity in x-direction
    x5 = SX.sym('x5') # yaw angle

    x = vertcat(x1, x2, x3, x4, x5)

    # U : controls
    u1 = SX.sym('u1')  # u1 = steering angle velocity of front wheels
    u2 = SX.sym('u2')  # u2 = longitudinal acceleration

    u = vertcat(u1, u2)

    # x_dot : states derivative
    x1_dot = SX.sym('x1_dot') 
    x2_dot = SX.sym('x2_dot') 
    x3_dot = SX.sym('x3_dot') 
    x4_dot = SX.sym('x4_dot') 
    x5_dot = SX.sym('x5_dot') 

    xdot = vertcat(x1_dot, x2_dot, x3_dot, x4_dot, x5_dot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    model.x0 = np.array([0.001, 0, 0, 0, 0])


    f1 = x4 * cos(x5)
    f2 = x4 * sin(x5)
    f3 = u1 
    f4 = u2 
    f5 = x4 / l_wb * tan(x3)

    f_expl = vertcat(f1, f2, f3, f4, f5)

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    
    return model