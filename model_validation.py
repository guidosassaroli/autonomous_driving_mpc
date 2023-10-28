from casadi import *
import numpy as np
import matplotlib.pyplot as plt
from pylab import *

from bicycle_model import bicycle_model


Ts = 0.025 # discretization time [s]
T = 8.00  # maximum simulation time [s]
N = int(T/Ts)  # number of discretization steps

# load prediction model
pred_model = bicycle_model()

xdot_mapping = Function('xdot_mapping', [pred_model.x,pred_model.u], [pred_model.f_expl_expr], ['x','u'], ['ode'])

## Configure Integrator
dae = {}
dae['x']   = pred_model.x    # states
dae['p']   = pred_model.u    # fixed during the integration horizon
dae['ode'] = xdot_mapping(pred_model.x,pred_model.u) # right-hand side
intg_options = {}
intg_options['tf'] = Ts
intg_options['simplify'] = True
intg_options['number_of_finite_elements'] = 4
intg = integrator('intg', 'rk', dae, intg_options) # Runge-Kutta 4 integrator
x1 = intg(x0=pred_model.x0, p =[1,0])     # try getting next state 

print('The next states are:', x1)

# -------------------------------------------- Plotting -------------------------------------------------
# Create a simulation model: (x0,U) -> (x1:xN): create a simulation model that generates 
# next states starting from x0 and applying series of control inputs U
sol_int = intg(x0=pred_model.x, p =pred_model.u)      # Simplify API to (x,u) -> (x_next) : general integrator solution mapping
x_next = sol_int['xf']
x_next_mapping = Function('x_next_mapping', [pred_model.x,pred_model.u], [x_next], ['x','u'], ['ode'])
simulation_model = x_next_mapping.mapaccum(N)

## Run a simulation trial
# U = cos([linspace(2,N,N), linspace(2,N,N)])   # control input series
v_delta = np.concatenate((linspace(0.01, 0, int(N/2)), linspace(0, -0.01, int(N/2))), axis = None)
U = np.array([v_delta, ones(N)])
# U = np.array([ones(N)+1, zeros(N)+np.deg2rad(250)])

sim_res = simulation_model(pred_model.x0, U)
velocity_state_evolution = np.array(sim_res[3,:])
velocity_state_evolution = velocity_state_evolution.reshape(-1)

print('Simulation results:')
print(sim_res)

## Plot results
t_grid = linspace(0, T, N)
fig = plt.figure()
plt.title("open loop prediciton model simulation")
ax1 = fig.add_subplot(221)
ax1.set_title("control inputs")
ax1.set_ylabel('acceleration [m/s²]', color='r')
ax1.step(t_grid, U[1,:], color='r')
ax1.tick_params(axis ='y', labelcolor = 'r')

ax1t = ax1.twinx()
ax1t.set_ylabel('steering velocity [°/s]', color='g')
ax1t.step(t_grid, np.rad2deg(U[0,:]), color='g')
ax1t.tick_params(axis ='y', labelcolor = 'g')
ax1.grid(True)


ax2 = fig.add_subplot(222)
ax2.set_title("position evolution")
ax2.plot(np.array(sim_res[0,:]).reshape(-1), np.array(sim_res[1,:]).reshape(-1))
ax2.set_ylabel('y [m]')
ax2.yaxis.tick_right()
ax2.yaxis.set_label_position("right")
ax2.set_xlabel('x [m]')
ax2.grid(True)

plt.subplot(223)
plt.title("velocity")
plt.plot(t_grid, np.array(sim_res[3,:]).reshape(-1))
plt.ylabel('v [m/s]')
plt.xlabel('t [s]')
plt.grid(True)

ax3 = fig.add_subplot(224)
ax3.set_title("orientation")
ax3.plot(t_grid, np.rad2deg((np.array(sim_res[4,:]).reshape(-1)+np.pi)%(2*np.pi)-np.pi))
ax3.set_ylabel('yaw [°]')
ax3.yaxis.tick_right()
ax3.yaxis.set_label_position("right")
ax3.set_xlabel('t [s]')
ax3.grid(True)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()