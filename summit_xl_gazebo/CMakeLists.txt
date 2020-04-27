cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_gazebo)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_srvs
    std_msgs
    tf
    gazebo_ros
    joint_state_controller
    velocity_controllers
    gazebo_ros_control
)


###################################
## catkin specific configuration ##
###################################
catkin_package(

)


#############
## Install ##
#############


install(
  DIRECTORY launch worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


