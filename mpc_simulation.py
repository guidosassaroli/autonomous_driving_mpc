import casadi as ca
import numpy as np
import json 

# -------------------- MPC DEFINITION ---------------------------

# Load reference trajectory 
X = np.genfromtxt("/home/guido/autonomous_driving_mpc/generated_trajectory.csv",delimiter=",",dtype=None)
simX = X[:, [0, 1, 2]]

x_ref = simX[:,0]
y_ref = simX[:,1]
yaw_ref = simX[:,2]
# v_ref = ref_traj_set['ref_v']

# ref_trajectory = [x_ref, y_ref, yaw_ref]
ref_trajectory = ca.MX(x_ref), ca.MX(y_ref), ca.MX(yaw_ref)
ref_trajectory = ca.vertcat(*ref_trajectory)




# Define symbolic variables and parameters as before

# MPC parameters
N = 10  # Prediction horizon
Q = np.diag([1, 1, 0.1, 0.1])  # State cost matrix
R = 0.01  # Control cost
num_steps = 10


# Define decision variables over the prediction horizon
X = ca.MX.sym('X', 4, N+1)  # State variables
U = ca.MX.sym('U', 1, N)  # Control inputs

# Create symbolic state and control vectors
X_vec = ca.reshape(X, -1, 1)

# Define the cost function
cost = 0
# for k in range(N):
#     cost += ca.mtimes([(X[:, k] - ref_trajectory).T, Q, (X[:, k] - ref_trajectory)]) + R * (U[:, k]**2)
for k in range(N):
    cost += ca.mtimes([(X[:, k] - ref_trajectory[:, k]).T, Q, (X[:, k] - ref_trajectory[:, k])]) + R * (U[:, k]**2)
    

# Define the optimization problem
g = []
g += [X[:, 0] - x_ref[:]]  # Initial state constraint
for k in range(N):
    x_next = ca.rhs_func(0, *X[:, k], U[:, k])
    g += [X[:, k+1] - x_next]

nlp = {'x': ca.vertcat(X_vec, U), 'f': cost, 'g': ca.vertcat(*g)}
opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-4, 'ipopt.hessian_approximation': 'limited-memory'}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# MPC control loop
state = [x_ref[0], y_ref[0], 0, 0, 0]
steering = 0
reference_state = [10, 2, 0, 5]  # Desired final state

for step in range(num_steps):
    t_val = step * ca.ode_opts['tf']
    inputs = [t_val] + state + [steering]
    
    # Solve the MPC problem
    lbx = [-np.inf] * (4 * (N + 1) + N)  # Lower bounds on variables
    ubx = [np.inf] * (4 * (N + 1) + N)   # Upper bounds on variables
    ubx[4 * (N + 1):] = [0.5] * N  # Control input limits

    res = solver(lbx=lbx, ubx=ubx, lbg=[0] * (4 * (N + 1)), ubg=[0] * (4 * (N + 1)))

    # Extract the first control input
    steering = res['x'][-N]

    # Simulate the system with the new control input
    state = ca.integrator(x0=state, p=inputs).full()

