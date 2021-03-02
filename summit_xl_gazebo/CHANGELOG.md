# Changelog for package summit\_xl\_gazebo

## Melodic devel (2021-03-01)
- Removed the summit_xl_robot_local_control launch node `summit_xl_one_robot.launch`
- Removed the summit_xl_perception launch node `summit_xl_one_robot.launch`
- Removed dependancy on `summit_xl_robot_control`
- Added run dependancy of `imu_complementary_filter`

## 1.1.3 (2018-05-15)

-   updated rviz config
-   gazebo: updating deps and maintainers
-   Add parameters to change navigation and localization mode
-   \[summit\_xl\_gazebo\] spawner of controllers moved to
    summit\_xl\_control package
-   added arg gui to launch gazebo with gui or not
-   added the build depend of hector\_gazebo\_plugin
-   config files of control moved to summit\_xl\_control pkg
-   allow to launch worlds of external pkgs
-   minor changes
-   sim: setting the omni drive configuration for SummitXL Steel
-   gazebo: adding rviz config for summit XLS
-   sim\_bringup: updating launch files and deleting old ones
-   gazebo: remapping params of launch files and new one for gazebo &
    rviz
-   gazebo: dependency of hector\_gazebo\_plugins
-   gazebo: adding param enable\_odom\_tf to the config
-   gazebo: minor changes
-   urdf model changed to summit\_xl\_std
-   \[summit\_xl\_gazebo\]:map frame now is /robot\_id\_map
-   \[summit\_xl\_gazebo\]:some tabs in launch file
-   \[summit\_xl\_gazebo\]:added gmapping include option
-   \[summit\_xl\_gazebo\]:launch file updated to multirobot\]
-   \[summit\_xl\_gazebo\]: yaml files updated with prefix
-   \[summit\_xl\_gazebo\]:rviz config updated
-   \[summit\_xl\_gazebo\]:rviz config added of office map
-   \[summit\_xl\_gazebo\]:summit\_xl\_multi.launch modified to 3 robots
-   added multirobot config and launch files
-   Merge branch 'kinetic-devel' into kinetic-multirobot-devel
-   Merge branch 'indigo-devel' into kinetic-devel
-   multi-robot launch modified
-   added rviz config
-   added launch for a single robot
-   added diff\_drive plugin controller configuration
-   summit\_xl\_sim multirobot branch
-   summit\_xl modified to multirobot
-   summit\_xl modified to multirobot
-   gazebo: adding flag to enable/disable the fake transform from map to
    odom
-   gazebo: update of rviz config
-   gazebo: adding pad to multi launch
-   gazebo: adding rviz config
-   adding gazebo launch for multirobots
-   updated simulation with summit\_xl\_robot\_control
-   summit\_xl\_sim: removed temp files
-   1.0.10

## 1.0.10 (2016-08-24)

## 1.0.9 (2016-07-13)

## 1.0.8 (2016-07-12)

## 1.0.7 (2016-07-12)

## 1.0.6 (2016-07-04)

## 1.0.5 (2016-07-01)

## 1.0.4 (2016-07-01)

-   modified dependencies
-   Contributors: carlos3dx

## 1.0.3 (2016-07-01)

-   Removed dependencies
-   Contributors: carlos3dx

## 1.0.2 (2016-07-01)

## 1.0.1 (2016-06-28)

-   Some spring cleaning
-   modified CMakeLists.txt and added urls and maintainers to package
    files
-   minor changes
-   added summit\_xl\_hls\_omni.launch file
-   launch pad from gazebo launch
-   Move base launch created. Still not working
-   Mux added. Old files removed.
-   using summit\_xl.urdf.xacro instead of previous standalone, using
    common pad
-   modified to use summit\_xl\_standalone.urdf.xacro robot file
-   chnaged summit\_xl.ga