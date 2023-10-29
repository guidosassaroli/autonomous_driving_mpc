# MPC for Self Driving Car
Model Predictive Controller (MPC) for self driving car. Implemented using [Casadi](https://web.casadi.org/).  

## Bicycle Model 

The bicycle model is describe in the file *bicycle_model.py*. 

States:  

$x$ = x position in a global coordinate system  
$y$ = y position in a global coordinate system  
$\delta$ = steering angle of front wheels  
$v$ = linear velocity  
$\phi$ = yaw angle  

Parameters:  

$l_{wb}$ = vehicle wheelbase  

Inputs:  

$v_{\delta}$ = steering angle velocity of front wheels  
$a$ = linear acceleration  

Differential equations:  

$\dot{x} = v cos(\phi)$    
$\dot{y} = v sin(\phi)$    
$\dot{\delta} = v_{\delta}$  
$\dot{v_x}= a$  
$\dot{\phi}= \frac{v}{l_{wb}} tan(\delta)$  

## Validation 

## Model Predictive Controller 
