summit_xl_sim
=============

Packages for the simulation of the Summit XL

<h2>summit_xl_control</h2>

This package contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager. It allows to launch the joint controllers for the Summit XL (4 axes skid steering + 2 axes ptz), Summit XL OMNI (4 axes skid steering, 4 axes swerve drive).

The Summit XL simulation stack follows the gazebo_ros controller manager scheme described in
http://gazebosim.org/wiki/Tutorials/1.9/ROS_Control_with_Gazebo

<h2>summit_xl_localization</h2>

This package contains launch files to use the EKF of the robot_localization package with the Summit XL robots. It contains a node to subscribe to gps data and publish it as odometry to be used as an additional source for odometry.

<h2>summit_xl_navigation</h2>

This package contains all the configuration files needed to execute the AMCL and SLAM navigation algorithms in simulation.

<h2>summit_xl_pad</h2>

This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform. The joystick output is feed to a mux (http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)



