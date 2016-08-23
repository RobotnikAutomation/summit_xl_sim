cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_robot_control)

find_package(catkin REQUIRED COMPONENTS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    tf
    )



###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    tf
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(summit_xl_robot_control src/summit_xl_robot_control.cpp)
add_executable(summit_xl_robot_control_vrep src/summit_xl_robot_control_vrep.cpp)

add_dependencies(summit_xl_robot_control robotnik_msgs_generate_messages_cpp)
add_dependencies(summit_xl_robot_control_vrep robotnik_msgs_generate_messages_cpp)


target_link_libraries(summit_xl_robot_control 
   ${catkin_LIBRARIES}
 )

target_link_libraries(summit_xl_robot_control_vrep
   ${catkin_LIBRARIES}
 )


#############
## Install ##
#############

install(
  TARGETS summit_xl_robot_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
