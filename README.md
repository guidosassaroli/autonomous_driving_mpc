# MPC for Self Driving Cars
Model Predictive Controller (MPC) for self driving car.

## States
$x$ = x position in a global coordinate system  
$y$ = y position in a global coordinate system  
$\delta$ = steering angle of front wheels  
$v_x$ = velocity in x-direction  
$\phi$ = yaw angle  

$l_{wb}$ = vehicle wheelbase  

$v_{\delta}$ = steering angle velocity of front wheels  
$a_x$ = longitudinal acceleration  



$\dot{x}$ = v_x * cos(phi)*  
$\dot{y}$ = v_x * sin(phi)*  
$\dot{\delta}$ = v_delta*  
$\dot{v}_x$ = a_x*  
$\dot{\phi}$ = v_x / l_wb * tan(delta)*  
