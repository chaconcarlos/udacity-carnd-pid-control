# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
---
##Overview

In this project, is necessary to build a PID controller that will handle the inputs from the Udacity's SDCE Term 2 simulator. For this, the hyperparameters for the controller should be tuned so the simulation can run as intended, with the car staying in the driveable portion of the road.

##Project Rubic Reflection

###Describe the effect each of the P, I, D components had in your implementation.

* **Proportional gain:** Describes the change in the output to reduce the error reported by the system. In this case, the cross-track error (CTE) that describes how far from the center of the road is the vehicle compared with the trajectory followed by the steering taken as input. The proportional gain accounts for the present values of the error, and how large will be the change to the original input. If the error is large, the change is going to be large. This value had the most direct effect in the vehicles steering, causing large corrections to the trajectory (for example, is the error indicated that the car was to far on the left of the center the road, a large correction to the right will be applied).
* **Integral gain:** This value describes the change of the output based on the cumulative sum of the past errors. This value has a subtle change, making the steering more smooth on paths were the error can be too big causing a disproportional response from the proportional gain value (like in the exit of the bridge in the simulator). This value compensates for bias in the system, that are constant errors that affect the ouput of the system. By being the cumulative sum of the all the CTE, it will grow large enough to eventually correct the output, if the proportional response is not sufficiently strong to correct it.
* **Derivative gain:** This gain takes in account the rate of change of the error, to avoid possible future trends of the error. This is represented as the tendency to overshoot and oscillate greatly trying to find the correct path. The derivative gain compensates the trend and reduces the oscillation, making the output to converge smoothly with the expected trajectory.

###Describe how the final hyperparameters were chosen.

The final parameters are `P = 0.389545 `,`I = 0.01`, `D = 7.757`. This values were first tuned by hand, so the twiddle process can give more accurate values, given that the wrong parameters can make the car steer out of the path making the run useless, and making the twiddle process longer and costly.

After getting values that made possible the car in the simulator to go around the track without steering out of the road, I implemented the twiddle process. Every run was set as a track around the track (or 1185 meditions from the simulator) and after each run ended, the simulator was restarted to "train" the weights with consistent input from the same conditions. The best error  plateaued around 300 laps, and after that I did some manual tuning.

The video of the laps with final parameters can be found in the next link.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=lRMiXlFzydA" target="_blank"><img src="http://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a> 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
