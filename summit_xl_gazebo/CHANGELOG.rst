^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package summit_xl_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-05-15)
------------------
* updated rviz config
* gazebo: updating deps and maintainers
* Add parameters to change navigation and localization mode
* [summit_xl_gazebo] spawner of controllers moved to summit_xl_control package
* added arg gui to launch gazebo with gui or not
* added the build depend of hector_gazebo_plugin
* config files of control moved to summit_xl_control pkg
* allow to launch worlds of external pkgs
* minor changes
* sim: setting the omni drive configuration for SummitXL Steel
* gazebo: adding rviz config for summit XLS
* sim_bringup: updating launch files and deleting old ones
* gazebo: remapping params of launch files and new one for gazebo & rviz
* gazebo: dependency of hector_gazebo_plugins
* gazebo: adding param enable_odom_tf to the config
* gazebo: minor changes
* urdf model changed to summit_xl_std
* [summit_xl_gazebo]:map frame now is /robot_id_map
* [summit_xl_gazebo]:some tabs in launch file
* [summit_xl_gazebo]:added gmapping include option
* [summit_xl_gazebo]:launch file updated to multirobot]
* [summit_xl_gazebo]: yaml files updated with prefix
* [summit_xl_gazebo]:rviz config updated
* [summit_xl_gazebo]:rviz config added of office map
* [summit_xl_gazebo]:summit_xl_multi.launch modified to 3 robots
* added multirobot config and launch files
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel' into kinetic-devel
* multi-robot launch modified
* added rviz config
* added launch for a single robot
* added diff_drive plugin controller configuration
* summit_xl_sim multirobot branch
* summit_xl modified to multirobot
* summit_xl modified to multirobot
* gazebo: adding flag to enable/disable the fake transform from map to odom
* gazebo: update of rviz config
* gazebo: adding pad to multi launch
* gazebo: adding rviz config
* adding gazebo launch for multirobots
* updated simulation with summit_xl_robot_control
* summit_xl_sim: removed temp files
* 1.0.10

1.0.10 (2016-08-24)
-------------------

1.0.9 (2016-07-13)
------------------

1.0.8 (2016-07-12)
------------------

1.0.7 (2016-07-12)
------------------

1.0.6 (2016-07-04)
------------------

1.0.5 (2016-07-01)
------------------

1.0.4 (2016-07-01)
------------------
* modified dependencies
* Contributors: carlos3dx

1.0.3 (2016-07-01)
------------------
* Removed dependencies
* Contributors: carlos3dx

1.0.2 (2016-07-01)
------------------

1.0.1 (2016-06-28)
------------------
* Some spring cleaning
* modified CMakeLists.txt and added urls and maintainers to package files
* minor changes
* added summit_xl_hls_omni.launch file
* launch pad from gazebo launch
* Move base launch created. Still not working
* Mux added. Old files removed.
* using summit_xl.urdf.xacro instead of previous standalone, using common pad
* modified to use summit_xl_standalone.urdf.xacro robot file
* chnaged summit_xl.gazebo summit_xl_fotonic.gazebo
* minor changes in gazebo files
* fixing package dependencies
* summit_xl_joystick: fixing ptz movement
* deleting old launch files
* updating summit_xl.world
* adding scissor movement
* summit_xl_description: creating first elements of Summit X robot
* fixing summit XL omni movement. adding omni plugin for gazebo
* deleting summit_xl_hq version, default summit_xl will be hq
* Not anymore multi.
* First models working on Rviz and Gazebo.
* Visulization in Gazebo almost done. Chassis origin and STL's corrected.
* 1.0.0
* Deleting summit_xl_joint_state. Creating summit_xl_sim metapackage
* summit_xl_2dnav: updating amcl packages and adding new map of Willow Garage
* summit_xl_2dnav: Updating config files to work with move base
* Fixed ptz controller error
* Allows multiple simulations of summit xl (not omni wheels yet)
* Added tf_prefix to urdf links
* Fixed omni movement
* Fixed more files
* Updated launch files and gazebo files
* summit_xl_gazebo. Fixed sintaxis error in summit_xl_multi.launch
* added first summit_xl_multi.launch
* initial commit with v305 of svn
* Contributors: Dani Carbonell, JorgeArino, RobotnikRoman, carlos3dx, dani-carbonell, mcantero, rguzman, trurl

* added summit_xl_hls_omni.launch file
* launch pad from gazebo launch
* Move base launch created. Still not working
* Mux added. Old files removed.
* using summit_xl.urdf.xacro instead of previous standalone, using common pad
* modified to use summit_xl_standalone.urdf.xacro robot file
* chnaged summit_xl.gazebo summit_xl_fotonic.gazebo
* minor changes in gazebo files
* fixing package dependencies
* summit_xl_joystick: fixing ptz movement
* deleting old launch files
* updating summit_xl.world
* adding scissor movement
* summit_xl_description: creating first elements of Summit X robot
* fixing summit XL omni movement. adding omni plugin for gazebo
* deleting summit_xl_hq version, default summit_xl will be hq
* Not anymore multi.
* First models working on Rviz and Gazebo.
* Visulization in Gazebo almost done. Chassis origin and STL's corrected.
* 1.0.0
* Deleting summit_xl_joint_state. Creating summit_xl_sim metapackage
* summit_xl_2dnav: updating amcl packages and adding new map of Willow Garage
* summit_xl_2dnav: Updating config files to work with move base
* Fixed ptz controller error
* Allows multiple simulations of summit xl (not omni wheels yet)
* Added tf_prefix to urdf links
* Fixed omni movement
* Fixed more files
* Updated launch files and gazebo files
* summit_xl_gazebo. Fixed sintaxis error in summit_xl_multi.launch
* added first summit_xl_multi.launch
* initial commit with v305 of svn
* Contributors: Dani Carbonell, JorgeArino, RobotnikRoman, dani-carbonell, rguzman, trurl
