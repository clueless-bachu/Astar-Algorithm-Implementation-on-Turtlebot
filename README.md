# Implementation of A star algorithm

This project is a part of the coursework for ENPM661: Planning of Autonomous Robots. This project has been completed in various phases.

### PHASE 1

ROS was installed in this phase.

### PHASE 2

In this phase, we had to implement A * algorithm on a rigid robot. The code is written in C++ in Astar_rigid.cpp.


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
The code is written in C++ in Astar_diff_phase3.cpp.

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

To run the program run ```./astar``` in the command line.

##### Video simulation



### PHASE 4
In this phase, Turtlebot3 is simulated in Gazebo.

##### Dependencies
- Ubuntu 16.04
- ROS Kinetic
- Gazebo
- Turtlebot3 Packages

##### Install Dependencies
- This project was developed using ROS Kinetic.
-	It is highly recommended that ROS Kinetic is properly installed on your system before the use of this project.
-	Follow the instructions on the ROS kinetic install tutorial page to install Full-Desktop Version of ROS Kinetic. The full-version would help you install Gazebo as well. 
-	Ensure successful installation by running Gazebo via your terminal window:
```
gazebo
```
-	Make sure that turtlebot3 packages have been installed on your machine using the following commands:
``` 
roslaunch turtlebot3_gazebo turlebot3_empty_world.launch
```
This should launch a window of Gazebo simulator with the turtlebot. If an error pops up upon launching, install the necessary turtlebot3 packages.

##### Build
Switch to your _src_ subdirectory of your ROS workspace to clone this repository.
```
<ROS workspace>/src
```
To build, run the following commands :
.....


.....
Run on terminal: 
```
cd <catkin workspace>
catkin_make
```
This will create an executable named ```astar```.


##### Running the program 

Run the following commands on the terminal: 
```
cd <ROS workspace>
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_map_p4.launch
```

Run the following commands on another terminal: 
```
cd <catkin workspace>
source devel/setup.bash
rosrun astar astar
```

##### Video simulation
