summit_xl_sim
=============

Packages for the simulation of the Summit XL

<a href="url"><img src="https://www.roscomponents.com/273-big_default/summit-xl.jpg" align="left" height="275" width="275" ></a>

<a href="url"><img src="https://www.roscomponents.com/279-big_default/summit-xl-steel.jpg" align="left" height="275" width="275" ></a>
<br />
<br />
<br />

<a href="url"><img src="https://www.robotnik.es/web/wp-content/uploads/2014/04/summit_xl_hl_sim.png" align="left" height="150" width="275" ></a>


<br />
<br />
<br />
<br />
<br />
<br />
<br />


<h1> Packages </h1>

<h2>summit_xl_gazebo</h2>

Launch files and world files to start the models in gazebo

<h2>summit_xl_sim_bringup</h2>

Launch files that launch the complete simulation of the robot/s



<h1>Simulating Summit XL</h1>

1. Install the following dependencies:
  - summit_xl_common [link](https://github.com/RobotnikAutomation/summit_xl_common)
  - robotnik_msgs [link](https://github.com/RobotnikAutomation/robotnik_msgs)
  - robotnik_sensors [link](https://github.com/RobotnikAutomation/robotnik_sensors)

    In the workspace install the packages dependencies:
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```  

2. Launch Summit XL simulation (1 robot by default, up to 3 robots): <br>
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

  ```
  Optional robot arguments:
  ```
  <!--arguments for each robot (example for robot A)-->
  <arg name="id_robot_a" default="summit_xl_a"/>
  <arg name="launch_robot_a" default="true"/>
  <arg name="map_file_a" default="willow_garage/willow_garage.yaml"/>
  <arg name="localization_robot_a" default="true"/>
  <arg name="gmapping_robot_a" default="false"/>
  <arg name="move_base_robot_a" default="false"/>
  <arg name="amcl_and_mapserver_a" default="false"/>
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

