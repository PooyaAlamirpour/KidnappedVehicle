## Self-Driving Car: Kidnapped Vehicle

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
Particle filters are a set of Monte Carlo algorithms used to solve filtering problems arising in signal processing and Bayesian statistical inference. The filtering problem consists of estimating the internal states in dynamical systems when partial observations are made, and random perturbations are present in the sensors as well as in the dynamical system.
In this project, assume a car is “kidnapped”. We just have its GPS location but it is noisy. Then the vehicle starts to move, in the meanwhile, it records the noisy sensor. Now, our mission is to estimate the location of the car by using Particle Filters. For building and testing this project, follow as below steps.

### Runing
Be consider before cloning and building the project, you should download the related [simulator](https://github.com/udacity/self-driving-car-sim/releases) which is made by the [Udacity](http://www.udacity.com). 
```bash
git clone https://github.com/PooyaAlamirpour/KidnappedVehicle.git
cd KidnappedVehicle
make build
cd build
./particle_filter
```
Once the code has been built and run successfully, run the simulator. The result is depicted as below:

![Output](https://github.com/PooyaAlamirpour/KidnappedVehicle/blob/master/Images/Output.png)

### Project structure
The directory structure of this repository is as follows:
```bash
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|___data
|   |   
|   |   map_data.txt
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```
The only file you should modify is `particle_filter.cp`p in the src directory. The file contains the scaffolding of a ParticleFilter class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do. If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Reference
[Particle filter](https://en.wikipedia.org/wiki/Particle_filter)
