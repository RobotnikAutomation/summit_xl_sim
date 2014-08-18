summit_xl_sim
=============

Packages for the simulation of the Summit XL, Summit XL HL and Summit-X (including X-WAM) robots

<h2>bh262_description</h2>

Contains the description of the Barrett Hand: urdf and meshes. This Hand is mounted on the WAM arm and is the end effector used by the X-WAM mobile manipulator.

<h2>summit_xl_2dnav</h2>

This package contains all the configuration files needed to execute the AMCL and SLAM navigation algorithms in simulation.

<h2>summit_xl_control</h2>

New gazebo 1.9 style robot control

<h2>summit_xl_description</h2>

robot description (urdf and meshes). Includes also V-REP model.

<h2>summit_xl_gazebo</h2>

launch files and world files to start the models in gazebo

<h2>summit_xl_joint_state</h2>

test node to publish joint states (as alternative to the joint_state_publisher)

<h2>summit_xl_joystick</h2>

<p>node to process the joystick in simulation (configured for PS3, but others are also possible).</p>

<h2>summit_xl_robot_control</h2>

<p>control the robot joints in all kinematic configurations, publishes odom topic and, if configured, also tf odom to base_link. Usually takes as input joystick commands and generates as outputs references for the gazebo controllers defined in summit_xl_control.</p>

<h2>summit_xl_waypoints</h2>

<p>pass a set of goals from a file to the move_base stack.</p>

<h2>wam_description</h2>

Barrett WAM (Whole Arm Manipulator) urdf and meshes.

<h2>xl_terabot_description</h2>

Robot description of the XL-Terabot robot.

<h2>x_wam_moveit</h2>

Draft moveit package to control the X-WAM.

