# autonomous_driving_mpc
Model Predictive Controller (MPC) for self driving car.

## States
*x* = x position in a global coordinate system  
*y* = y position in a global coordinate system  
*delta* = steering angle of front wheels  
*v_x* = velocity in x-direction  
*phi* = yaw angle  
*l_wb* = vehicle wheelbase  

*v_delta* = _steering angle velocity of front wheels_
*a_x* = _longitudinal acceleration_


*x_dot = v_x * cos(phi)*
*y_dot = v_x * sin(phi)*
*delta_dot = v_delta*
*v_x_dot = a_x*
*phi_dot = v_x / l_wb * tan(delta)*