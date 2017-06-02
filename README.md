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

There was some care taken to choosing these values.  When the timestep was too small the car was unstable.  I actually had started with timestep of 0.1 and with some trial and error eventually increased it to 0.15 in order to keep the car stable.  A timestep of 0.2 wasn't fine enough for the turns.  And the timestep of 0.1 was unstable.

Therefore, after choosing the smallest possible time step which covers the latency of 100ms:

```
double dt = 0.15;
size_t N = 10;
```

The size N was initially set to 20 but it was decreased by trial and error to 10.

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

A new feature has been added in an attempt to take latency into account correctly.  The idea here is that I have already have the core MPC implemented correctly.  I just need to add one small thing which takes latency into account.  And since the MPC which I have created assumes there is no latency, meaning the initial state is at time=0.  So, what if I could just change that initial state to time=0.1?

Therefore, I have used the global kinematics equations to calculate what would be the future state (at time=0.1) and errors with the following code:

```
double dt = 0.1;

// Global Kinematics equations
double px_future = px + v*cos(psi)*dt;
double py_future = py + v*sin(psi)*dt;
double psi_future = psi - v*(delta/Lf)*dt;   // Note that there is a negative sign because delta is negative
double v_future = v + a*dt;

// Calculating the future cte and episi based on the new future positions above
double cte_future = polyeval(coeffs, px_future) - py_future;
double epsi_future = -atan(coeffs[1]);
```

Next, I am back to turning parameters.  It is still is obvious to seek to minimize cte and epsi to zero.  There was some tuning required to strengthen these parameters.  

```
double tune_cte =  2000;
double tune_epsi = 5000;
```

Note that the speed cannot be minimized to zero since the car may come to a complete stop and be stuck in that state.  Therefore a reference speed must be set.  It is set to 40 in this case.  Anything higher runs the risk of touching the red and white bumps!  

Then the cost of using actuators (steering and acceleration) were added and tweaked.  

```
double tune_delta = 1000;
double tune_a = 25;
```

The cost of steering was increased to minimize random erratic driving such as random oscillations.  Also the cost of acceleration was increased to minimize constant accelerating and braking of the car.

Then the final cost added was to minimize the sequential steering and acceleration.  

```
double tune_dseq = 500;
double tune_aseq = 25;
```

Next, the model constraints was set up based on the vehicle model equations stated in the model section of the MPC udacity lecture notes.

The solve function was primarily borrowed from the Udacity MPC mind the line code and modified.  Basically, this section was used set boundaries for the state variables, steering and acceleration.

Finally, the the ipopt solver is called and the steering value and acceleration is calculated to steer and accelerate the car

This MPC should have no problems with latency as the car was able to drive around the track without going off the road.  The car is actually able to make it around the track even up to 40mph.  I'm sure that this would also make the CHP (California Highway Patrol) happy that we stay within the speed limits.

The issue of latency was dealt with by the way of implementing this MPC in such a way that the timestep was at least 0.1 seconds (set to 0.15) and feeding in the future state into the solver!  This was definitely a big improvement, the car is able to drive around the road at a higher speed than the initial release.

#### New improvements 2017-06-02
In addition to fixing latency there are some other improvements to this release:

Corrected steering angle as follows:
```
steer_value = -1*(vars[6]/deg2rad(25));
```

Correctly calculate the 3rd order polynomial:
```
// 3rd degree polonomial f(x)
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
// tangential angle is calculated as arctan(f'(x)) above
AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0);
```


## Future work
It is concluded that this project is a success and that the car is able to drive around the track as specified.  Some future challenges would be to increase the speed of the car and keep it on the track using MPC.
