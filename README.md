# Fuzzy Control with AR mark Tracking for Parrot Bebop Drone 

## Pre-requisites
* Operation System
  * Ubuntu 16.04
  
* Middleware 
  * [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Kinetic)

This package depends on:
* [ar_tools package](http://wiki.ros.org/ar_tools)
* [bebop_autonomy package](https://bebop-autonomy.readthedocs.io/en/latest/index.html)

## Getting started 

Make sure ROS is correctly installed

1. Create and initialize the workspace
  
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
4. Turn on your drone
   
   ```
    Wait until the drone booted
    Connect your computer to the drone's access point
    ```
 
5. Launch the Bebop Driver

   ```
   roslaunch bebop_driver bebop_node.launch
   ```
6. Takeoff your Drone
  
   ```
   rostopic pub -1 /bebop/takeoff std_msgs/Empty
   ```
 
## Autonomous AR_mark Tracking - Fuzzy Controller ##

1. Open a new terminal and Run the ar_pose package launch
   
   ```
   cd ~/bebop_ws/src
   source devel/setup.bash
   roslaunch mark_track ar_pose_bebop.launch
   ```
   
A window of rviz will open automatically. You will see something like this:


![](https://github.com/dvalenciar/Fuzzy_Control_with_AR_mark_Tracking-Bebop2/blob/master/imageRviz.png)

2. Download and print the ar_tag mark. Available [here](https://github.com/dvalenciar/Fuzzy_Control_with_AR_mark_Tracking-Bebop2/blob/master/4x4_384_20.gif)

   ```
   Move the ar_mark pointing to the Drone's camera. You will see how the algorithm detects the mark even if you rotate      it.
   ```

3.  Run the fuzzy controller (**Be careful, Drone will start moving.**) 

    If the drone detects the ar_mark it will move and track the ar_mark AUTONOMOUSLY. In the case of not detecting any mark,     the drone will maintain its position
   
   
   ```
   rosrun mark_track position_track_fuzzy.py
   ```
   
## Autonomous AR_mark Example Video ##

https://www.youtube.com/watch?v=7wDhm-8efFE&t=3s

![](https://github.com/dvalenciar/Fuzzy_Control_with_AR_mark_Tracking-Bebop2/blob/master/pic.png)

