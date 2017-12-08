## Implementation

### The Model

The implementation for the Model Predictive Controller resides in lines 119-124 of MPC.cpp. It is written along with a legend of the model's state variables. 

```
fg[x_start    + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[y_start    + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[psi_start  + t + 1] = psi1 - (psi0 - v0/Lf * delta * dt);
fg[v_start    + t + 1] = v1 - (v0 + a * dt);
fg[cte_start  + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[epsi_start + t + 1] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);
```
- The values of `x0, x1` and `y0, y1` correspond to the car's position
- The values of `psi0, psi1` correspond to the car's heading
- The values of `v0, v1` corresponds to the car's velocity
- The value of `cte1` corresponds to the car's cross-track error
- The values of `epsi0, epsi1` correspodn to the car's error of orientation
- The value of `a` corresponds to the car's acceleration
- The value of `delta` corresponds to the car's steering angle

In lines 54 to 69, the acceleration/throttle variable `a` and steering angle variable `delta` are found after minimization.

Much trial and error in tuning the variables for the timestep length and elapsed duration, `N` and `dt`, respectively, had to be made. If the timestep length were set to 15 to 20, the tracking line would move further ahead of the car during simulation. Also, the greater the number of points, the poorer the performance of the simulation. Lowering the timestep number to 10 helped greatly in this regard. As far as the elapsed duration, anything greater than 0.1 caused the car to sway very rapidly, causing the car to veer off track and dip into the water/sink into the sand dunes. For best results, limit the timestep to between 10 and 15 and limit the elapsed duration to between 0.05 and 0.1.

In main.cpp, lines 104 to 111 deal with setting and preprocessing the MPC with the given waypoints while in lines 119-126, the polynomial coefficients are found after polynomial fitting. The `steering_value` and `throttle` are found and help with the drawing of the reference trajectory of the car.

Lastly, the latency issue is dealt with in lines 187 to 188 of main.cpp. The thread sleeps 100ms as a way to combat latency.
