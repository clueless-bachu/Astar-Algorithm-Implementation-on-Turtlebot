# Implementation of A star algorithm

This project is a part of the coursework for ENPM661: Planning of Autonomous Robots. This project has been completed in various phases.

### PHASE 1

ROS was installed in this phase.

### PHASE 2

In this phase, we had to implement A * algorithm on a rigid robot. The code is written in C++ in astar_rigid.cpp


##### Dependencies

Install Opencv for C++ from this link https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

Note:
- This installation is for Ubuntu system.
- The steps where "optional" is mentioned are optional.
- Save the opencv source code in the home directory.

##### Compiling

- It is assumed you have cmake and make installed in your ubuntu system.
- Go to the location of this directory.
- Run the following commands on terminal:  
```
cd Implementation-of-Astar-algorithm
cmake ./
make
```
- This creates an executable named ```astar```.

##### Running the program

To run the program run ```./astar``` in the command line


##### Video simulation

A video of the output can be seen at https://youtu.be/wbpHSscLVFs.

### PHASE 3

In this phase, we changed the rigid robot to Turtle bot and had to implement A* algorithm for non holonomic constraints. The code takes the start coordinate (x_s, y_s, theta_s), the goal coordinate (x_g, y_g) and two values of "RPM" for the wheel velocities as input from the user and creates an optimal path for the robot to reach the goal.

##### Dependencies

Install Opencv for C++ from this link https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

Note:
- This installation is for Ubuntu system.
- The steps where "optional" is mentioned are optional.
- Save the opencv source code in the home directory.

##### Compiling



##### Running the program



##### Video simulation



### PHASE 4
In this phase, the Turtlebot is simulated in Gazebo.

##### Dependencies


##### Compiling


##### Running the program 


##### Video simulation
