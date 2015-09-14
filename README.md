summit_xl_sim
=============

Packages for the simulation of the Summit XL

![Image of Summit XL](http://www.robotnik.es/web/wp-content/uploads/2014/03/summit-xl-robots-moviles-robotnik_s01.jpg)

<h2>summit_xl_control</h2>

New gazebo 1.9 style robot control. Contains the controllers and launch files to start them in gazebo.

<h2>summit_xl_gazebo</h2>

launch files and world files to start the models in gazebo

<h2>summit_xl_robot_control</h2>

<p>control the robot joints in all kinematic configurations, publishes odom topic and, if configured, also tf odom to base_link. Usually takes as input joystick commands and generates as outputs references for the gazebo controllers defined in summit_xl_control. This package permits an alternative way to control the robot motion (4 motorwheels) that by default is carried on by the Gazebo plugin (skid-steer). In the default configuration this package only controls the pan-tilt camera joints.

When used as main controller of the simulated robot, this node also computes the odometry of the robot using the joint movements and a IMU and publish this odometry to /odom. The node has a flag in the yaml files that forces the publication or not of the odom->base_footprint frames, needed by the localization and mapping algorithms.
</p>

<h2>summit_xl_sim_bringup</h2>

launch files that launch the complete simulation of the robot
