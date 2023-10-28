# MPC for Self Driving Car
Model Predictive Controller (MPC) for self driving car.

## Bicycle Model 

The bicycle model is describe in the file *bicycle_model.py*. 

States:  

$x$ = x position in a global coordinate system  
$y$ = y position in a global coordinate system  
$\delta$ = steering angle of front wheels  
$v_x$ = velocity in x-direction  
$\phi$ = yaw angle  

Parameters:  

$l_{wb}$ = vehicle wheelbase  

Inputs:  

$v_{\delta}$ = steering angle velocity of front wheels  
$a_x$ = longitudinal acceleration  

Differential equations:  

$\dot{x} = v_x cos(\phi)$    
$\dot{y} = v_x sin(\phi)$    
$\dot{\delta} = v_{\delta}$  
$\dot{v_x}= a_x$  
$\dot{\phi}= \frac{v_x}{l_{wb}} tan(\delta)$  

## Validation 

## Model Predictive Controller 
