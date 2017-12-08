# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


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
