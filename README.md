# summit_xl_sim

Packages for the simulation of the Summit XL

<p align="center">
  <img src="doc/summit_xl.jpeg" height="275" />
  <img src="doc/summit_xl_steel.jpeg" height="275" />
  <img src="doc/summit_xl_gazebo.png" height="275" />
  <img src="doc/summit_xl_steel_gazebo.png" height="275" />
</p>

This packages contains: 

## summit_xl_gazebo

Launch files and world files to start the models in gazebo

## summit_xl_sim_bringup

Launch files that launch the complete simulation of the robot/s

## Simulating Summit XL

This simulation has been tested using Gazebo 9 version.

## Installation and run instruccions

### 1. Install the following dependencies:

 To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

### 2. Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
```

Install the latest version of the simulation:

```bash
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/summit_xl_sim/melodic-devel/repos/summit_xl_sim_devel.repos
```

**Install the ROS dependencies**

```bash
rosdep install --from-paths src --ignore-src --skip-keys="summit_xl_robot_control marker_mapping robotnik_locator robotnik_pose_filter robotnik_gazebo_elevator" -y -r
```

<!--
For the stable version (some latest features may be not available):

```bash
vcs import --input \
  https://raw.githubusercontent.com/RobotnikAutomation/summit_xl_sim/melodic-master/doc/summit_xl_sim.repos
rosdep install --from-paths src --ignore-src --skip-keys="summit_xl_robot_control" -y
``` -->

### 3. Compile:

```bash
catkin build
source devel/setup.bash
```

**ONLY: if catkin build doesn't work:** The package catkin-tools is need to compile with catkin build:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

### 4. Launch Summit XL simulation (1 robot by default, up to 3 robots):

#### Summit XL:

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch
```

#### Summit XL with Trossen Arm

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch default_xacro:=summit_xl_tix_std.urdf.xacro launch_arm_a:=true arm_manufacturer_a:=trossen arm_model_a:=vx300s
```

Launch moveit to plan trajectories:

```bash
ROS_NAMESPACE=robot roslaunch summit_xl_vx300s_moveit_config demo.launch
```

#### Summit XL with Kinova Arm

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch default_xacro:=summit_xl_gen_std.urdf.xacro launch_arm_a:=true arm_manufacturer_a:=kinova arm_model_a:=j2s7s300 amcl_and_mapserver_a:=false move_base_robot_a:=false
```

**Note:** in this configuration the robot has not laser, therefore the amcl is turned off. When Rviz is opened, change robot_map to robot_odom in ```fixed_frame```  in order to visualize the robot.

#### or Summit XL Steel:

```bash
roslaunch summit_xl_sim_bringup summit_xls_complete.launch
```

#### Optional general arguments:

```xml
<arg name="launch_rviz" default="true"/>
<arg name="gazebo_world" default="$(find summit_xl_gazebo)/worlds/summit_xl_office.world"/>
<arg name="omni_drive" default="false"/> (only for Summit XL)
<arg name="use_gpu_for_simulation" default="false"/>
```

By default the Gazebo plugin [Planar Move](http://gazebosim.org/tutorials?tut=ros_gzplugins) to ignore the physics of the wheels + the skid steering kinematics. In case you want to disable this plugin, set the following arguments:

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch \
  ros_planar_move_plugin:=false \
  omni_drive:=false
```

#### Optional robot arguments:

```xml
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

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch \
  launch_robot_b:=true \
  launch_robot_c:=true
```

- Example to launch simulation with 1 Summit XL robot with navigation:

```bash
roslaunch summit_xl_sim_bringup summit_xl_complete.launch \
  move_base_robot_a:=true \
  amcl_and_mapserver_a:=true
```

Enjoy! You can use the topic `${id_robot}/robotnik_base_control/cmd_vel` to control the Summit XL robot or send simple goals using `/${id_robot}/move_base_simple/goal`

## Docker usage

### Installation of required files

#### Intel GPU

- [docker engine](https://docs.docker.com/engine/install/ubuntu/)
- [docker compose plugin](https://docs.docker.com/compose/install/linux/)

#### Nvidia GPU

- [docker engine](https://docs.docker.com/engine/install/ubuntu/)

- [docker compose plugin](https://docs.docker.com/compose/install/linux/)

- nvidia-drivers

- [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

### Usage

```bash
git clone https://github.com/RobotnikAutomation/summit_xl_sim.git
cd summit_xl_sim
git checkout melodic-devel
export ROS_BU_PKG="summit_xl_sim_bringup"
export ROS_BU_LAUNCH="summit_xl_complete.launch"
nvidia-smi &>/dev/null \
&& ln -sf docker-compose-nvidia.yml docker-compose.yml \
|| ln -sf docker-compose-intel.yml docker-compose.yml
docker compose up
```

#### Selecting the robot model

You can select the robot, the launch file of package using the optional arguments on launch
By default the selected robot is `summit_xl`

**Summit XL GEN**

```bash
export ROS_BU_PKG="summit_xl_sim_bringup"
export ROS_BU_LAUNCH="summit_xl_gen_complete.launch"
docker compose up
```

**Summit XLS**

```bash
export ROS_BU_PKG="summit_xl_sim_bringup"
export ROS_BU_LAUNCH="summit_xls_complete.launch"
docker compose up
```

#### Manual Build

If you wish to build the image without launching the simulation use the following commands:

```bash
cd docker
docker compose build
```

#### Notes

This is docker requires a graphical interface

- In order to exit you have to 2 options

- The `ROS_MASTER_URI` is accessible outside the container, so in the host any ros command should work

- You could also run a `roscore` previous to launch the simulation in order to have some processes on the host running

- if you want to enter on the container use the following command in another terminal
1. Close `gazebo` and `rviz` and wait a bit

2. execute in another terminal:
   
   ```bash
   docker container rm --force summit_xl_sim_instance
   ```

#### Notes

- This is docker requires a graphical interface

- The `ROS_MASTER_URI` is accessible outside the container, so in the host any ros command should work

- You could also run a `roscore` previous to launch the simulation in order to have some processes on the host running

- if you want to enter on the container use the following command in another terminal
  
  ```bash
  docker container exec -it docker-base-1 bash
  ```

- In order to exit you have to 2 options
1. Close `gazebo` and `rviz` and wait a bit

2. execute in another terminal in the same folder than the `docker-compose.yml`:
   
   ```bash
   docker compose down
   ```