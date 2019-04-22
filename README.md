# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./images/update_eq.png "Update equation"
[image2]: ./images/youtube.png "Youtube"
---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Project Rubrics

### The Model
I use MPC model with the following states (all in vehicle coordinate system):
  - `x`: Car position x 
  - `y`: Car position y
  - `psi`: Car heading
  - `v`: Car velocity
  - `cte`: Cross track error; the error of position y from reference line.
  - `epsi`: Orientation error.

The actuators are steering and throttle, which maps to `delta` and `a`.

The update equations is based on course material with slight modification: since in simulator the steering angle is in the opposite direction to the car heading, the equation negates the "delta" part (with red circle).

![Screenshot][image1]


### Timestep Length and Elapsed Duration (N & dt)
I have tried different combinations of N and dt separately. I first experiment dt in range [0.01, 0.5] with step 0.01. It turns out dt=0.1 gives me the best simulation result. Other dt values makes the car swing dramatically and instable.

I also experiement different values of N. Since N cannot be to small (otherwise the horizon is too short) neither too large (seems acutation errors will accumulte). I pick range of N in [5, 20]. It turns out N=10 gives me acceptable result.

The final decision is `N=10`, `dt=0.1`.


### Polynomial Fitting and MPC Preprocessing
Polynomial fitting is done by the following steps (`MPC.cpp` line 58 ~ 76)

1. Convert simulator message's `ptsx` and `ptsy` from world coordinate system to vehicle coodinate system. This is done by:
    - Transit by `(-px, -py)`
    - Rotate by `-psi`
2. Truncate the waypoints since we only need to fit adequate lengths of reference line. In this case, since we use N=10 and dt=0.1, a reference line of future 1 second movement is enough. So I truncate waypoints by `MAX_POLY_FIT_NUM=10` (1 second / 100ms).

3. Fit truncated waypoints with polynomial order=`3`


### Model Predictive Control with Latency
The latency is handled by using "delayed acutations" in model updating (`MPC.cpp` line 111~117). The idea is let the model update happens at T<sub>i</sub> uses the actuation at T<sub>i-1-n_delay</sub> instead of T<sub>i-1</sub> to mimic the latency in acutation.

`n_delay` is the additional delay, computed by `latency/dt`. In this project,n_delay=1 (0.1/0.1).


### Parameter tuning.
The most time consuming part is to tune parameters used in cost function. The Cost function is composed by several parts, each part has a corresponding parameter to address the criticalness in penalizing the optimizer to achieve desiered behaviors.

- cte (kCTE) 
- epsi (kEPSI)
- speed reference (kRefV)
- steering (kDelta) 
- throttle (kA)
- steering times throttle (kDeltaCrossA) 
- sequental change of steering (kSeqDelta)
- sequential change of throttle (kSeqA)


Here I describe my tuning process:

1. I started with all paramters = 1.0. The car immediate goes out of track at the beginning straight track.

2. To keep car in track, I found that increase the `kEPSI` gives the best result. By suing kEPSI=100 the car can finish the lap.

3. However, at sharp turns or sequential turns (s-curve), the car sometimes will ride on curb (but can recover it). This means the car does not honor reference line in such situation. So I increase the value of `kCTE`. Increasing kCTE will force the car tends to stay on reference line, however, it will cause overdamping.

4. To reduce overdamping, I need to penalize the car when its heading changes too aggresively, so I increase `kEPSI` and also `kSeqDelta`. I found an empirical rule: when KCTE increase, kEPSI and kSeqDelta also needs to be increased to keep the car stable.

5. Morever, I found increase `kDeltaCrossA` further helps the car to stay on renference line in sharp turn: because it will try to do brake when the speed is high at sharp turn. It figures out to lower the speed to avoid too large steering! However, the car tends to brake too much (from 60 mph to 15mph).

6. The rest I tuned is to increase `kSeqA` to keep the car not doing aggresive breaking at sharp turn. Now the car can pass sharp turns at 25+ mph.

7. Final parameters:

    | Params   | Value |
    |----------|-------|
    | kCTE     | 600    |
    | kEPSI    | 600    |
    | kRefV    | 1    |
    | kDelta   | 1 |
    | kA       | 1 |
    | kDeltaCrossA  | 300 |
    | kSeqDelta | 800 |
    | kSeqA    | 50 |



### The vehicle must successfully drive a lap around the track.
The simulation result shows the vehicle is stayed on drivable area. At some sharp turns it will infringe the curb lanes but still keep on track.

[YouTube video](https://www.youtube.com/watch?v=-QoZ7TFy0Rs)

![Screenshot][image2]
