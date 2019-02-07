# Fuzzy Control with AR mark Tracking for Parrot Bebop Drone 

## Pre-requisites
* Operation System
  * Ubuntu 16.04
  
* Middleware 
  * [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Kinetic)

This package depends on:
* [ar_tools_package](http://wiki.ros.org/ar_tools)
* [bebop_autonomy package](https://github.com/AutonomyLab/bebop_autonomy)
  * For more Information about this package, please visit this [page](https://bebop-autonomy.readthedocs.io/en/latest/index.html) 

## Getting started 

Make sure ROS is correctly installed

1. Create and initialize the workspace (if you already have  bebop_autonomy package  installed you can omit this part)
  
   ``` 
   mkdir -p ~/bebop_ws/src
   cd ~/bebop_ws/src
   catkin_init_workspace
   ``` 
2. Download dependencies and built the package
   
   ``` 
   git clone https://github.com/AutonomyLab/bebop_autonomy.git                 #Bebop Drone ROS Driver
   git clone https://github.com/ar-tools/ar_tools.git                          #ar_pose ROS package
   git clone https://github.com/dvalenciar/Fuzzy_Control_with_AR_mark_Tracking-Bebop2.git
   cd..
   rosdep update
   rosdep install --from-paths src -i
   catkin_make
   ``` 
   
3. Source the environment
   
   ```
   source devel/setup.bash
   ```
   
4. Launch the Driver

   ```
   roslaunch bebop_driver bebop_node.launch
   
   ```
   
## Autonomous AR_mark Tracking - Fuzzy Controller ##


   
   
