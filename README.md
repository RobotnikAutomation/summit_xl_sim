summit_xl_sim
=============

Packages for the simulation of the Summit XL

<p align="center">
  <img src="https://github.com/RobotnikAutomation/summit_xl_sim/blob/melodic-master/doc/summit_xl.jpeg" height="275" />
  <img src="https://github.com/RobotnikAutomation/summit_xl_sim/blob/melodic-master/doc/summit_xl_steel.jpeg" height="275" />
  <img src="https://github.com/RobotnikAutomation/summit_xl_sim/blob/melodic-master/doc/summit_xl_gazebo.png" height="275" />
  <img src="https://github.com/RobotnikAutomation/summit_xl_sim/blob/melodic-master/doc/summit_xl_steel_gazebo.png" height="275" />

</p>

<h1> Packages </h1>

<h2>summit_xl_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>summit_xl_sim_bringup</h2>

Launch files that launch the complete simulation of the robot/s

<h1>Simulating Summit XL</h1>

This simulation has been tested using Gazebo 9 version.

1. Install the following dependencies:

 To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

2. Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/summit_xl_sim/melodic-master/doc/summit_xl_sim.repos
rosdep install --from-paths src --ignore-src -y
```

3. Compile:

```bash
catkin build
source devel/setup.bash
```

Note: The package catkin-tools is need to compile with catkin build:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

4. Launch Summit XL simulation (1 robot by default, up to 3 robots): <br>
- Summit XL: <br>
  ```
  roslaunch summit_xl_sim_bringup summit_xl_complete.launch
  ```

- or Summit XL Steel: <br>
  ```
  roslaunch summit_xl_sim_bringup summit_xls_complete.launch
  ```
  Optional general arguments:
  ```
  <arg name="launch_rviz" default="true"/>
  <arg name="gazebo_world" default="$(find summit_xl_gazebo)/worlds/summit_xl_office.world"/>
  <arg name="omni_drive" default="false"/> (only for Summit XL)
  <arg name="use_gpu_for_simulation" default="false"/>
  ```
  By default the Gazebo plugin [Planar Move](http://gazebosim.org/tutorials?tut=ros_gzplugins) to ignore the physics of the wheels + the skid steering kinematics. In case you want to disable this plugin, set the following arguments:
  ```
  roslaunch summit_xl_sim_bringup summit_xl_complete.launch ros_planar_move_plugin:=false omni_drive:=false
  ```

  Optional robot arguments:
  ```
  <!--arguments for each robot (example for robot A)-->
  <arg name="id_robot_a" default="robot"/>
  <arg name="launch_robot_a" default="true"/>
  <arg name="map_file_a" default="willow_garage/willow_garage.yaml"/>
  <arg name="localization_robot_a" default="false"/>
  <arg name="gmapping_robot_a" default="false"/>
  <arg name="amcl_and_mapserver_a" default="true"/>
  <arg name="x_init_pose_robot_a" default="0" />
  <arg name="y_init_pose_robot_a" default="0" />
  <arg name="z_init_pose_robot_a" default="0" />
  <arg name="xacro_robot_a" default="summit_xl_std.urdf.xacro"/>
  ```
- Example to launch simulation with 3 Summit XL robots:
  ```
  roslaunch summit_xl_sim_bringup summit_xl_complete.launch launch_robot_b:=true launch_robot_c:=true
  ```
- Example to launch simulation with 1 Summit XL robot with navigation:
  ```

  roslaunch summit_xl_sim_bringup summit_xl_complete.launch move_base_robot_a:=true amcl_and_mapserver_a:=true
  ```
3. Enjoy! You can use the topic "${id_robot}/robotnik_base_control/cmd_vel" to control the Summit XL robot or send simple goals using "/${id_robot}/move_base_simple/goal"
