# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Predictive Control

### The project 
The project consists in driving a car in a simulator based on an MPC model.
The simulator input data (and model output) are: 


- steering and throttle data

The model input (and simulator output) are:

 - the path guideline
 - the vehicle kinematic parameters
	 - position x, y, orientation
	 - speed v, steering angle 

### The Model

Here are the main steps of the program (main.cpp file):

- the "path guideline" is expressed in the vehicle local coordinates
then approximated by a parabola (second degree polynomial).
- the vehicle characteristics after a latency period (actuator time) are computed based on current vehicle characteristics.

.

    double x_lat 		= v * latency;
    double y_lat		= v * sin(epsi0) * latency;      
    double psi_lat 		= - v * steering / Lf * latency;    
    double v_lat		= v;    				
    double cte_lat 		= cte0 + v * sin(epsi0) * latency;    
    double epsi_lat 	= epsi0 + psi_lat;	

	    

- from this latency position, a trajectory is computed by minimizing a cost function defined relative to the "path guideline" .
- the first step of this trajectory is transferred to the simulator as a throttle and a steering value.

The vehicle model is hidden into the optimization parameters (MCP.cpp file) :

- It is a kinematic model, the vehicle dynamic are not considered at this stage (inertia forces, tyre behaviour, steering-suspension model,...).


- in the computation of the successive steps of the trajectory


	- x ​t+1 ​= x​t +v​t ∗ cos(ψ​t)∗dt
	- y ​t+1​​ = y​t​​ +v​t​​ ∗ sin(ψ​t)∗dt
	- ψ ​t+1​​ = ψ​t​​ +​L​f ∗ ​v​t ∗ δ ∗ dt
	- v ​t+1​​ = v​t + a ∗dt
	- cte ​t+1 = cte ​t + v​t ∗sin(eψ​ t) ∗ dt
	- eψ ​t+1​​ =eψ ​t ​+ ​L​f ∗ ​v​t ∗ δ​t ∗dt

- in the optimisation constraints
	- limiting steering angle to [-25,25] deg
	- limiting acceleration to [-1,1]




- in the definition of the cost parameters for the optimisation as a function of :
	- cross track error
	- orientation error
	- gap to target speed
	- throttle gap
	- steering gap
	- throttle continuity
	- steering continuity

###hyperparameters
####timeframe and steps

Here are the selected time parameters
`size_t N = 10;
double dt = 0.05;`
it corespond to a time horizon of 0.5s

N=10 leads to a fast computing time.

### cost parameters

the cost function is expressed as follow: 
   ` cost = w_cte*cte^2 + w_epsi*epsi^2 + w_dv*(v - ref_v) + w_d_delta*(delta(t+1)-delta(t))^2 + w_d_a*(a(t+1)-a(t))^2 + w_a*a^2 + w_delta*delta^2`

I selected the following parameters:

    /// init target speed
    #define ref_v 		80

    /// hyperparameters for the cost function
    #define w_cte 		5		// cross track error
    #define w_epsi 		1		// direction error
    #define w_dv 		1		// gap to target speed
    #define w_a 		1		// throtle gap
    #define w_delta 	100		// steering gap
    #define w_d_a 		0		// throtle continuity factor
    #define w_d_delta 	1e6	// steering continuity factor

The parameters:



- weights w_cte, w_epsi, w_dv are used to balance cte, epsi and distance to target speed,


- weights w_d_delta and w_d_a are used to control smoothness steering, smoothness of acceleration,


- weights w_delta and w_a are used to minimize the use of steering and the use of acceleration.

The tuning of the "cost" hyper-parameters was done manually, the process was as follow :

- 1. Select a very large weight for the "steering continuity" to provide a smooth trajectory.
- 2. Select moderate weight for the "target speed parameter".
- 3. Increase the weight of the `cte` to keep the optimisation close to the guide line.

The tuning of the parameter is a try and error process, with multiple possible choices.

##Result
The car run an ~ 80 mph, some further tuning of the hyper parameters can be performed, but the selected values seem to provide a relatively smooth drive and keep the car on the road.
