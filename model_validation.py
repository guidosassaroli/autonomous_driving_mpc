from casadi import *
import numpy as np
import matplotlib.pyplot as plt
from pylab import *
import pandas as pd 

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
v_delta = np.concatenate((linspace(0, 0.01, int(N/2)), linspace(0.01, 0, int(N/2))), axis = None)
v_delta = 0.05*ones(N)

U = np.array([v_delta*0, ones(N)*0])

sim_res = simulation_model(pred_model.x0, U)
velocity_state_evolution = np.array(sim_res[3,:])
velocity_state_evolution = velocity_state_evolution.reshape(-1)

print('Simulation results:')
print(sim_res)

## ------------------ Plot result ----------------------------
t_grid = linspace(0, T, N)

plt.figure('States')
plt.subplot(221)
plt.plot(t_grid, U[0,:], 'b')
ax = plt.gca()
plt.xlabel('time [s]')
plt.ylabel('Steering rate')
plt.grid(axis='both')

plt.subplot(222)
plt.plot(t_grid, U[1,:], 'r')
ax = plt.gca()
plt.xlabel('time [s]')
plt.ylabel('Longitudinal acceleration')
plt.grid(axis='both')

plt.subplot(223)
plt.plot(t_grid, np.array(sim_res[3,:]).reshape(-1), 'b')
ax = plt.gca()
plt.xlabel('time [s]')
plt.ylabel('longitudinal velocity [m/s]')
plt.grid(axis='both')

plt.subplot(224)
plt.plot(t_grid, np.rad2deg((np.array(sim_res[4,:]).reshape(-1)+np.pi)%(2*np.pi)-np.pi), 'b')
ax = plt.gca()
plt.xlabel('time [s]')
plt.ylabel('yaw angle [°]')
plt.grid(axis='both')

plt.show()


plt.figure('Driven trajectory')
plt.plot(np.array(sim_res[0,:]).reshape(-1), np.array(sim_res[1,:]).reshape(-1), 'b')
# plt.plot(sim_res[0,:], sim_res[1,:], 'b')
ax = plt.gca()
plt.xlabel('x pos [m]')
plt.ylabel('y pos [m]')
plt.grid(axis='both')
plt.show()


# ---------------- Save datas: sim_res, U, t ------------------------------
pos_x = np.array(sim_res[0,:]).reshape(-1)
pos_y = np.array(sim_res[1,:]).reshape(-1)
yaw = np.array(sim_res[4,:]).reshape(-1)

data = np.column_stack((pos_x, pos_y, yaw))
df = pd.DataFrame(data = data.astype(float))
df.to_csv('generated_trajectory.csv', sep=',', header = False, float_format='%.5f', index = False)

