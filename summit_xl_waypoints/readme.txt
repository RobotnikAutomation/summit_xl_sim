
Sending of waypoints to the move_base controller of the Summit_XL rover

*To test

Free indoor environment:
Configured to get odom estimation by means of the robot_pose_ekf filter. 

roslaunch summit_xl_waypoints simulation_environment.launch
roslaunch summit_xl_waypoints summit_xl_waypoints.launch



Outdoor environment:
roslaunch summit_xl_waypoints simulation_environment_gps.launch
roslaunch summit_xl_waypoints summit_xl_waypoints.launch



