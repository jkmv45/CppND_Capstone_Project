# CPPND Capstone  Project: Autonomous Swarm Simulator

## Description:
The project I have built is based on the simluation I created for my Master's thesis in 2014-2015 that simulated fixed-wing unmanned aerial vehicles (UAV) controlled to exhibit swarm or "flocking" behavior.  The simulation was originally written in MATLAB and it was very early in my engineering career so there were certainly some challenges porting it over to a C++ application.

Note: Not all of the functionality has been ported over yet, but the essentials are there.

The simulation will create a number of agents with random initial conditions within a certain volume in 3D space.  That volume is based on the sensing range of the agents with a goal that each agent can sense at least one other agent.  The flow of the simulation is as follows:

  time loop start

    agent loop start

      For Agent i:  

      SenseNeighbors()

      ComputeControl()

      PropagateStates()

    agent loop end

  time loop end

Each swarm agent is equipped with one sensor that can measure the pose and speed of its neighbors.  With that information, each agent can compute the forward acceleration and angular rates required to maintain consensus on their heading, speed, and separation distance.  Now the new states of each agent can be computed by integrating the controller output.  

The success of the swarm is judged based on the relative heading, speed, and distance between agents.  If all agents are heading the same direction at the same speed and near a target separation distance all within a specified tolerance, the swarm behavior has been achieved.  If one agent get separated from the group or a collision occurs, the objective is failed.

## Class Structure:
The project is broken into the following classes:

* Environment Manager
  * Basically holds all objects that may exist in the simulation environment.  
  * Executes the simulation and handles data that needs to be passed between objects.
  * Computes relative states between agents for logging purposes.
  * Computes the graph Laplacian which is a way of tracking which agents can communicate or sense each other.
* Swarm Agent
  * Represents a UAV in the swarm.  
  * The primary public method is the Simulate function.  This updates class data in a single simulation step.
* Sensor
  * Represents a generic sensor attached to each agent.
  * Obtains pose and speed data for neighboring agents from the environment for use by the owning agent.
* SimObj
  * Abstract Base Class (ABC) for a generic simulation object.  
  * For now, this only represents a swarm agent, but the intent is to hold all data/functions that may be used by other future objects like obstacles, uncooperative targets, etc.

## Run Instructions:
Running the application is rather simple. Once your run the application, you will be asked to select a number of vehicles (aka agents) to simulate.  

* NOTE: Currently the simulation is limited to five (5) agents, mainly to limit the execution time to a resonable value for this project and also to not generate plots with too many lines.

Next, the application will tell you approximately how much time the simulation is capturing (not execution time).  The application will give you a progress counter as a percentage.  

Finally, a summary of the results will be displayed that tells whether or not the simulation successfully achieved swarm behavior.  Then a set of three plots will pop up:
* Seperation Distance over time
* Relative Heading over time
* Relative Speed over time.

The plots are created with the CPP wrapper for matplotlib (MPL) in Python.  This allows for some basic plot manipulation like panning and zooming.  Once you are done viewing the plots, simply close them out to end the application.

## Expected Output:
Even though the initial conditions are randomized per run, the conditions and time duration should yield a passing result on all test criteria with plots reflecting those results.  A failed result may indicate that the time estimate was too short to achieve swarm behavior that meets the specifications.

## Rubric Criteria Satisfied:
### Loops, Functions, I/O: 
  (1)  Criteria: The project demonstrates an understanding of C++ functions and control structures.
      Satisfied by: Code is grouped into functions that perform specific tasks.  A variety of for loops and if/else statements are used for accessing data, making decisions, and progressing the simulation.
### Object Oriented Programming
  (2)  Criteria: The project uses Object Oriented Programming techniques.
      Satisfied by: My project is broken into four classes: EnvironmentManager, SwarmAgent, Sensor, and SimObj.
  (3)  Criteria: Classes use appropriate access specifiers for class members.
      Satisfied by: Each class specifies public, private, and protected data depending on the application.  SimObj is a parent class so its variables are protected for the sake of inheritance.
  (4)  Criteria: Classes encapsulate behavior.
      Satisfied by: Data and methods are grouped based on how it might look in a real application.  For example, all Swarm Agents will have at least one Sensor.  That sensor will return all of the data from agents inside its sensing range that exist in the environment.  So the sensor in this case acts as a buffer between each agent and the environment.  Data that must conform to strict properties are hidden from the user and use setters with invariants.  For example, the pose of an agent contains the position and attitude of that agent.  The attitude component is a 3x3 direction cosine matrix that must have a determinant of 1 to be a valid attitude.  The setter in this class will check if the pose being requested is valid before writing.  If the determinenat is slightly off from 1, it will renormalize the matrix, but if it is very far off, it will reject the data.
### Memory Management
  (5)  Criteria: The project makes use of references in function declarations.
      Satisfied by: The helper functions in SimObj use pass-by-reference to reduce the amount of data copied and/or operate on the referenced data.

## References: 
Mills, John. Swarm Coordination Scheme for Fixed-Wing UAVs. Florida Institute of Technology.  Master's Thesis.  2015.


## Dependencies for Running Locally
Developed and tested on: Linux Ubunto 22.04 (VM)

* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Eigen >= 3.4.0
  * Linux: sudo apt install libeigen3-dev
  * Mac:  Using Homebrew, run the following command in terminal: brew install eigen
  * Windows:  Download the desired release from http://eigen.tuxfamily.org.

              Unzip in the location of your choice, preferrably at C:\ or C:\Program files for better discoverability by CMake find-modules (remember to extract the inner folder and rename it to Eigen3 or Eigen).
* python3 (w/ numpy and matplotlib) >= 3.0
  * Linux:  sudo apt-get install python3-dev python3-numpy python3-matplotlib
  * Windows/Mac: [click here for installation instructions](https://realpython.com/installing-python/)
* matplotlibcpp
  * This package will be fetched by cmake if not found locally
* google test >= 1.12.1
  * This package will be fetched by cmake if not found locally

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run application: `./SwarwmSimulation`
5. Run tester: `./test/SwarmAgentTester`