# Navigation #


## Install ##

ROS navigation packages


    sudo apt-get install ros-kinetic-navigation



ROS package: gradient_based_navigation


    git clone https://github.com/Imperoli/gradient_based_navigation

add to catkin workspace and compile.


## Run ##

### Standard navigation ###

To launch navigation modules, use the script 

    python runnav.py

or start localizer and navigation modules manually

    roslaunch srrg_localizer.launch map_name:=<MAP> initial_pose_x:=<X> initial_pose_y:=<Y> initial_pose_a:=<THETA>
    roslaunch move_base.launch


To send target goals to the robot, use the script

    python move.py <GX> <GY> <GTheta DEG>

(GX,GY,GTheta): target pose in map coordinates


### Obstacle avoidance behavior only ###

To launch only the obstacle avoidance behavior:

    roslaunch obstacle_avoidance.launch


You need a controller (e.g., joystick teleop) to drive the robot.

Note: Controllers must publish on /desired_cmd_vel instead of /cmd_vel


