# CarND-Path-Planning-Project
**from Udacity's Self-Driving Car Engineer Nanodegree Program**

## Introduction
In this project I built a path planner, which creates safe and smooth trajectories for the vehicle to follow in a simulator. It considers other vehicles based on sensor fusion data and obeys the speed limit of 50 MPH.

![figure](pics/path_planning1.png)

## Implementation
The path planner is based on a Finite State Machine (FSM). The available states are:

* Keep Lane (*KL*)
* Preparing a Lane Change Left/Right (*PLCL*/*PLCR*)
* and Lane Change Left/Right (*LCL*/*LCR*)

 In each state it choses the best next state by means of several cost functions. The following features are considered in the cost functions:

* the target state itself:
  * penalize states *PLCL* and *PLCR* to avoid unnecessary lane changes
  * if already in states *PLCL*/*PLCR* or still in states *LCL*/*LCR* (lane change not finished yet) reward states *LCL* and *LCR* to avoid breaking up a lane change prematurely
* the target lane
  * prefer middle lane over outer lanes
  * prefer lanes that are closer to best lane
* speed of the target lane:
  * prefer faster lanes
* distance to the next vehicle in front and behind in the target lane:
  * prefer a higher distance
* number of vehicles in front in the target lane:
  * prefer less occupied lanes

With these cost functions the path planner can "decide" when it is better to change lanes.

In order to achieve smooth trajectories the waypoints are derived using a spline function.

## Rubric Points

### Valid Trajectories

The path planner complies with the following rubric points:
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit and as fast as possible considering the current traffic.
* The car does not exceed a total acceleration of 10 m/s² and a jerk of 10 m/s³.
* Car does not have collisions.
* The car is able to smoothly change lanes when it makes sense to do so.
* The car doesn't spend more than a 3 second length outside the lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

### Reflection

For the path planner I created a class `Vehicle` which is used to hold all data of the ego vehicle, the map data as well the sensor fusion data. When creating an instance of this class, the map data of the track are passed. The `update` method is used to feed the new data from the simulator to the class, i.e. the current location and speed of the ego vehicle, the residual of the previous path and the data from sensor fusion. Furthermore, the method calls `residual_feasible` which checks if the remainder of the previous path is still feasible, i.e. there is no other vehicle which suddenly crossed the way.

When the vehicle data is up to date and the residual path is safe to follow the `choose_next_state` method is called by the main program. This method choses the best next state from the list of possible next states based on their costs and returns the corresponding trajectory. First, the method invokes the method `generate_predictions` which calculates for each lane the distance to the closest vehicles, their velocity and the total number of cars in front of and behind the ego vehicle. Afterwards the method `successor_states` is called. It returns the possible next states for the current state in the Finite State Machine.

For each of the possible next states a rough trajectory is generated (`generate_trajectory`, `keep_lane_trajectory`, `prep_lane_change_trajectory` and `lane_change_trajectory`). If a state is not feasible an empty trajectory is returned and the state is not considered anymore in this loop. This can happen if another car is too close to the predicted position of the ego vehicle in the target lane after a lane change.

When calculating the waypoints for the trajectory, the speed and the distance to the next vehicle in front and behind is determined for the target lane based on the prediction data. The smaller the distance to the next cars and the lower their velocity the lower the chosen lane speed.

After accquireing all relevant information for each successor state, the costs for each state is determined. The cost functions are defined in the file `costs.cpp`. The state with the minimal cost is selected as next state.

Based on the waypoints of the rough trajectory for this state and the lane speed, a smooth trajectory is generated with the spline function from [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/). The local coordinate system of the ego vehicle was used. Otherwise it cannot be guaranteed that the x-values are in ascending order which is required by the spline function. The acceleration/deceleration is limited such that the maximal acceleration and jerk are not exceeded. Then, the coordinates are transformed back to the global coordinates and appended to the remainder of the previous path. Finally, the complete trajectory is returned and passed to the simulator.


---

The lines below are from the original README file of the [Udacity CarND-Path-Planning project](https://github.com/udacity/CarND-Path-Planning-Project)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
