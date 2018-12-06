# sailbots_simulator_control_test
MATLAB-ROS communication for control of an autonomous sailboat

This repository contains a MATLAB file that:

* connects to an existing ROS master
* continuously receives position, orientation, and velocity data from a Gazebo sailboat simulator
* continuously outputs thrust values to the three thrusters on the boat: left, right and lateral
* converts quaternions into euler angles for easier understanding
* converts net force and moment about the centre-of-mass to thrusts 

This repository is intended to testing the control system for UBC Sailbots. The control system will be receiving the position, orientation, and velocity data and will output a net force and moment to the boat. The conversion from the net force and moment to thrusts is shown by calculations in the image file in this repository.


__Note__: This MATLAB file requires the use of the Robotics Toolbox.
