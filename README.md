# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Purpose

The Purpose of this project is to explore the basics of an MPC controller and implement it in C++.  

## Implementation

### Model
The model state consist of the following:

my_state = [x y psi v cte epsi]

x = current car's x coordinate
y = current car's y coordinate
psi = current car's orientation
v = current car's velocity
cte = cross track error
epsi = orientation error

actuators =  [delta a]

delta = steering angle
a = accleration

The equations to calculate the next state according to the current state is as follows:

x1 = x0 + v0 * cos(psi0) * dt
y1 = y0 + v0 * sin(psi0) * dt
psi1 = psi0 + v0 * delta0 / Lf * dt
v1 = v0 + a0 * dt)
cte1 = ((f0 - y0) + (v0 * sin(epsi0) * dt))
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt)


### Timestep Length and Frequency

There was some care taken to choosing these values.  As too many small timesteps may make the calculation too slow and we wouldn't meet the latency requirement.  However, if the time step is too large we won't be able to capture a turn properly.

Therefore, I prioritized choosing the smallest possible time step which covers the latency of 100ms.  Therefore dt was chosen to be at least 0.1:
double dt = 0.1;
size_t N = 8;

The size N was initially set to 20 but it was decreased by trial and error to 10 and then finally to 8 to increase stability.

### Polynomial Fitting and MPC Preprocessing

Since the waypoints were provided in the map coordinates we must first convert these waypoints to vehicle coordinates.  To do this we must do some trigonometry.

The strategy is to loop through the list of way points and do the following:
1. Take way point x coordinate and subtract the car's current position x
	dx = ptsxc[i] - px;
2. Take way point y coordinate and subtract the car's current position y
	dy = ptsyc[i] - py;

  The above two steps will provide two sides of the triangle.  Now, based on the vehicle orientaton we can find where the point is relative to the vehicle.  Note that px and py are the vehicle's current car coordinates based on the map coordinates.  And the ptsxc and ptsyc are the way points based on map coordinates.

3.  ptsxc[i] = dx*cos(psi) + dy*sin(psi);
4.  ptsyc[i] = dy*cos(psi) - dx*sin(psi);

In the above two equations we calculate the position in vehicle coordinates based on the dx and dy which was calculated above and the vehicle orientation in map coordinates.  Finally, since we already made use of the px, py, and psi values we can now also map these to vehicle coordinates which happens to be:

px = py = psi = 0

Now we have a new vector of waypoints in vehicle coordinates (ptsxc and ptsyc).  They can be fed into a fitted polynomial as follows:

```
Eigen::VectorXd coeffs = polyfit(ptsxe, ptsye, 3);
double cte = polyeval(coeffs, px) - py;
double epsi = -atan(coeffs[1]);`
```

Also note that these coefficients are used to calculate the cte and epsi.

### MPC with latency

In the stage of MPC implementation, first, it is obvious to seek to minimize cte and epsi to zero.  There was some tuning required to strengthen these parameters.  But the speed cannot be minimized to zero since the car may come to a complete stop and be stuck in that state.  Therefore the cost for a reference speed must be set to a value which was 33 in this case.

Then the cost of using actuators (steering and acceleration) were added and tweaked.  The cost of steering was increased to prevent random erratic driving such as random oscillations.  Also the cost of acceleration was increased to prevent constant accelerating and braking of the car.

Then the final cost added was to nerf the sequential steering and acceleration.  These parameters were also slightly tuned.

Next, the model constraints was set up based on the vehicle model equations stated in the model section.

Moving on to the solve function.  Basically, just need to set boundaries for the state variables, steering and acceleration.

Finally, the the ipopt solver is called and the steering value and acceleration is calculated to steer and accelerate the car

This MPC should have no problems with latency as the car was able to drive around the track without going off the road.  The car is actually able to make it around the track even up to 40mph but the wheels would be entering the red and white areas.  Therefore the speed was lowered to about 31 mph to prevent the car from going into the red and white areas on the sharp turns.  This would also make the CHP (California Highway Patrol) more happy.

The issue of latency was dealt with by the way of implementing this MPC in such a way that the timestep was at least 0.1 seconds.  This is because the MPC is able to take a trajectory (which would ideally be gathered from map and localization information) to predict where the vehicle will go in the next few steps.  However, we are interested only in the first step after the initial step. Therefore, it was desired to have the actuation variables are calculated for the future step of 100 ms which accounts for the latency!

## Future work
It is concluded that this project is a success and that the car is able to drive around the track as specified.  Some future challenges would be to increase the speed of the car and keep it on the track using MPC.
